#include <stdio.h>
#include <stdint.h>
#include <math.h>


int main()
{
	FILE * f = fopen( "chirpbuff.h", "w" );
	FILE * fd = fopen( "chirpbuff.dat", "w" );
	double chirp_begin = 903.9-.075;
	double chirp_end = 903.9+.075;
	double sample_rate = 1040.0/6.0; // Sampler at 173MHz.
	double sampletotal = 8192*(32);
	fprintf( f, "const uint32_t chirpbuff[%d] = {\n", (int)(sampletotal/32) );
	int samples;

	// For a given word, it is shifted out MSB (Bit and byte) first

	uint32_t sample_word = 0;
	int samplect = 0;
	int words = 0;
	double phase = 0;
	for( samples = 0; samples < sampletotal; samples++ )
	{
		double current_f = (samples / sampletotal) * ( chirp_end - chirp_begin ) + chirp_begin;
		phase += 3.1415926 * 2.0 * current_f / sample_rate;
		int bit = sin( phase ) > 0.1;
		sample_word |= !!bit;
		samplect++;
		if( samplect == 32 )
		{
			fprintf( f, "0x%08x,%c", sample_word, (words & 0xf0)?' ':'\n' );
			fwrite( &sample_word, 1, 4, fd );
			words++;
			sample_word = 0;
			samplect = 0;
		}
		sample_word <<= 1;
	}

	fprintf( f, "};\n" );
	fclose( f );
	fclose( fd );
	fprintf( stderr, "Wrote out %d uint32_t's.\n", words );
}
