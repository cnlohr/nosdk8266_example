#include <stdio.h>
#include <stdint.h>
#include <math.h>

const uint32_t memory_offset = 0x20000;

const double center_frequency = 903.9;
const double chirp_begin = center_frequency-.075;
const double chirp_end = center_frequency+.075;

const double sample_rate = 1040.0/6.0; // Sampler at 173MHz.
const double chirp_length_seconds = 0.001024;
const double sampletotal = ( chirp_length_seconds * sample_rate * 1000000 ) + 0.5;

uint32_t bleedover = 256;
int words = 0;
int words_nominal = 0;

//FILE * fcba;
FILE * fd;
FILE * fCBI;


void GenChirp( double fStart,  double fEnd )
{
	uint32_t sample_word = 0;
	int samplect = 0;
	double phase = 0;
	int samples = 0;
	int ic;
	{
		for( samples = 0; ; samples++ )
		{
			double placeInSamples = samples  / sampletotal;
			if( placeInSamples >= 1 )
			{
				placeInSamples -= 1;
			}
			double current_f = ( placeInSamples ) * ( fEnd - fStart ) + fStart;
			phase += 3.1415926 * 2.0 * current_f / sample_rate;
			int bit = sin( phase ) > 0.8;
			sample_word |= !!bit;
			samplect++;
			if( samplect == 32 )
			{
				//fprintf( fcba, "0x%08x,%c", sample_word, (words & 0xf0)?' ':'\n' );
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
	//fcba = fopen( "chirpbuff.h", "w" );
	fd = fopen( "chirpbuff.dat", "w" );
	fCBI = fopen( "chirpbuffinfo.h", "w" );

	//fprintf( fcba, "const uint32_t chirpbuff[%d] = {\n", (int)(sampletotal/32) );

	// For a given word, it is shifted out MSB (Bit and byte) first
	GenChirp( chirp_begin, chirp_end );
	int sample_word_median = words_nominal;
	int quarter_chirp_length = ((sample_word_median+2)/4);

	words = 0;
	GenChirp( chirp_end, chirp_begin );

	//fprintf( fcba, "};\n" );
	//fclose( fcba );
	fclose( fd );
	fprintf( stderr, "Wrote out %d uint32_t's.\n", words + sample_word_median );

	fprintf( fCBI, "#define CHIRPLENGTH_WORDS (%d)\n", words_nominal );
	fprintf( fCBI, "#define MEMORY_START_OFFSET (0x%08x)\n", memory_offset );
	fprintf( fCBI, "#define REVERSE_START_OFFSET (0x%08x)\n", memory_offset + sample_word_median*4 );
	fprintf( fCBI, "#define QUARTER_CHIRP_LENGTH_WORDS (%d)\n",  (int)(quarter_chirp_length) );
	fprintf( fCBI, "#define CHIRPLENGTH_WORDS_WITH_PADDING (%d)\n", sample_word_median );
	fprintf( fCBI, "#define STRIPE_BLEEDOVER_WORDS (%d)\n", bleedover );

	int factor = 0;
	int i;
	// Minimum of 32, max of 255 words
	for( i = 32; i < 255; i++ )
	{
		if( quarter_chirp_length % i == 0 )
		{
			factor = i;
		}
	}
	if( !factor )
	{
		fprintf( stderr, "Error: No factor of %d found, you may have to do something clever in rf_data_gen.c\n", words_nominal );
		return -9;
	}
	fprintf( fCBI, "#define DMA_SIZE_WORDS (%d)\n", factor );
	fprintf( fCBI, "#define NUM_DMAS_PER_QUARTER_CHIRP (%d)\n", quarter_chirp_length/factor );
	fclose( fCBI );
}
