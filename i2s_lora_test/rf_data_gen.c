#include <stdio.h>
#include <stdint.h>
#include <math.h>

const double center_frequency = 904.1;
const double bw = .125;
const uint32_t memory_offset = 0x20000;

#define SF_NUMBER 7

#if SF_NUMBER < 7
#warning SF6 still does not work :(
#endif

#define SPI_DIV 1

// Funny modes:
/// 80MHz, SF8,  SPI_DIV 5 @903.9/904.1 produces hilarious mirror images around 904.0

#if MAIN_MHZ == 80
const double sample_rate = 1040.0/13.0/SPI_DIV;
#if ( SF_NUMBER > 8 ) 
#error Not enough ram for chirp table
#endif

#elif MAIN_MHZ == 115
const double sample_rate = 1040.0/9.0/SPI_DIV;
#if ( SF_NUMBER > 8 ) 
#error Not enough ram for chirp table
#endif

#elif MAIN_MHZ == 52
const double sample_rate = 1040.0/20.0/SPI_DIV;
#if ( SF_NUMBER > 9 ) 
#error Not enough ram for chirp table
#endif

#elif MAIN_MHZ == 173
const double sample_rate = 1040.0/6.0/SPI_DIV;
#if (  SF_NUMBER > 7 ) 
#error Not enough ram for chirp table
#endif

#else
#error Unknown Clock Rate
#endif

const double chirp_begin = center_frequency-bw/2;
const double chirp_end = center_frequency+bw/2;

const double chirp_length_seconds = (8<<SF_NUMBER) * 0.000001;
const double sampletotal = ( chirp_length_seconds * sample_rate * 1000000 );

int words = 0;
int words_nominal = 0;
uint32_t bleedover = 0; // Will be tailored based on the needed buffers.

FILE * fcba;
FILE * fd;
FILE * fCBI;


void GenChirp( double fStart,  double fEnd )
{
	uint32_t sample_word = 0;
	int samplect = 0;
	double phase = 0.0001;
	int samples = 0;
	int ic;
	{
		for( samples = 0; ; samples++ )
		{
			double placeInSamples = samples / sampletotal;
			if( placeInSamples >= 1 )
			{
				// When going off the top frequency, start over at the bottom.
				placeInSamples -= 1;
			}
			double current_f = ( placeInSamples ) * ( fEnd - fStart ) + fStart;
			phase += 3.1415926 * 2.0 * current_f / sample_rate;
			int bit = sin( phase ) > 0.0; // HINT: if you want to mix multiple signals, add a DC offset here.
			sample_word |= !!bit;
			samplect++;
			if( samplect == 32 )
			{
				fprintf( fcba, "0x%08x,%c", sample_word, (words & 0xf)?' ':'\n' );
				fwrite( &sample_word, 1, 4, fd );
				words++;
				sample_word = 0;
				samplect = 0;
				if( samples < sampletotal )
					words_nominal = words;

				if( samples >= sampletotal + bleedover*32 )
				{
					break;
				}
			}
			sample_word <<= 1;
		}
	}

}

int main()
{
	fcba = fopen( "chirpbuff.h", "w" );
	fd = fopen( "chirpbuff.dat", "w" );
	fCBI = fopen( "chirpbuffinfo.h", "w" );

	fprintf( fcba, "const uint32_t chirpbuff[] = {\n" );



	int quarter_chirp_length_bits = (int)(sampletotal/4.0+0.5);
	int factor = 0;
	int is_perfect_divisor = 0;
	int i;

	// DMA size words, when multiplied out should be SMALLER than the
	// overall length of our message, so that we can trim off the last few
	// words sometimes, to average out to the right bitrate.

	// So, we need to pick a number that when multiplied out, ends up
	// slightly larger than IDEAL_CHIRP_LENGTH_BITS.  But not too much so.

	// First try to find a perfect divisor, if we can find a perfect divisor,
	// then no weird fixup stuff needs to be done.

	int seeking_perfect_divisor = 1;
	bleedover = 0;

retry_without_seek:
	for( i = 511; i > 100; i-- )
	{
		int nr_to_div = (quarter_chirp_length_bits / 32 + 32) / i;
		int num_bits_default = i * 32 * nr_to_div;
		int leftover = num_bits_default - quarter_chirp_length_bits;
		//fprintf( stderr, "%d %d %d [%d] --> %d\n", i, nr_to_div, num_bits_default, quarter_chirp_length_bits, leftover );
		if( leftover < 32*nr_to_div &&
				( seeking_perfect_divisor ? (leftover == 0) : (leftover >= 0)) )
		{
			// Make sure we aren't more than 1 32-bit word too large
			fprintf( stderr, "Found %s divisor: %d (%d - %d = %d)\n",
				(leftover == 0)?"PERFECT" : "OK",
				i,
				num_bits_default,
				quarter_chirp_length_bits,
				num_bits_default - quarter_chirp_length_bits );
			factor = i;

			//XXX TODO: I think this bleedover calc is wrong. Should investigate?
			bleedover = num_bits_default/32;
			is_perfect_divisor = leftover == 0;
			break;
		}
	}

	if( seeking_perfect_divisor && !factor )
	{
		seeking_perfect_divisor = 0;
		goto retry_without_seek;
	}

	if( !factor )
	{
		fprintf( stderr, "Error: No factor of %d found, you may have to do something clever in rf_data_gen.c\n", quarter_chirp_length_bits );
		return -9;
	}

	// For a given word, it is shifted out MSB (Bit and byte) first
	GenChirp( chirp_begin, chirp_end );
	int sample_word_median = words;
	int quarter_chirp_length = ((words_nominal+2)/4);
	int reverse_start = words;
	words = 0;
	GenChirp( chirp_end, chirp_begin );

	fprintf( fcba, "};\n" );
	fclose( fcba );
	fclose( fd );
	fprintf( stderr, "Wrote out %d uint32_t's.\n", words + sample_word_median );


	fprintf( fCBI, "#define CHIRPLENGTH_WORDS (%d)\n", words_nominal );
	fprintf( fCBI, "#define MEMORY_START_OFFSET (0x%08x)\n", memory_offset );
	fprintf( fCBI, "#define REVERSE_START_OFFSET (0x%08x)\n", memory_offset + reverse_start * 4 );
	fprintf( fCBI, "#define QUARTER_CHIRP_LENGTH_WORDS (%d)\n",  (int)(quarter_chirp_length) );
	fprintf( fCBI, "#define IDEAL_QUARTER_CHIRP_LENGTH_BITS (%d)\n", quarter_chirp_length_bits );
	fprintf( fCBI, "#define CHIRPLENGTH_WORDS_WITH_PADDING (%d)\n", sample_word_median );
	fprintf( fCBI, "#define STRIPE_BLEEDOVER_WORDS (%d)\n", bleedover );
	fprintf( fCBI, "#define TARGET_SAMPLE_COUNT_BITS (%d)\n", (int)sampletotal );
	fprintf( fCBI, "#define DMA_SIZE_WORDS (%d)\n", factor );
	fprintf( fCBI, "#define NUM_DMAS_PER_QUARTER_CHIRP (%d)\n", quarter_chirp_length/factor+((is_perfect_divisor)?0:1) );
	fprintf( fCBI, "#define SF_NUMBER %d\n", SF_NUMBER );
	fprintf( fCBI, "#define SPI_DIV %d\n", SPI_DIV );
	if( is_perfect_divisor )
		fprintf( fCBI, "#define FOUND_PERFECT_DIVISOR\n" );
	fclose( fCBI );
}
