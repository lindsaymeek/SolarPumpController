
//----------------------------------------------------------------------------
//
// Fast binary square root for 16-bit integer argument. Contest entry C2982.
//
// Based on the non-restoring method http://www.dattalo.com/technical/theory/sqrt.html
//
// Copyright 2004-2007
//
//----------------------------------------------------------------------------


#include "isqrt.h"

unsigned char isqrt(unsigned long N)
{
	unsigned short s2 = 0;

	//
	// unrolled loop for faster execution
	//

   if(16384 <= N) { N-=16384; s2=32768; }

   N<<=1;

   if((8192+s2)<=N) { N-=8192+s2; s2|=16384; }

   N<<=1;

   if((4096+s2)<=N) { N-=4096+s2; s2|=8192; }

   N<<=1;

   if((2048+s2)<=N) { N-=2048+s2; s2|=4096; }

   N<<=1;

   if((1024+s2)<=N) { N-=1024+s2; s2|=2048; }

   N<<=1;

   if((512+s2)<=N) { N-=512+s2; s2|=1024; }

   N<<=1;

   if((256+s2)<=N) { N-=256+s2; s2|=512; }

   N<<=1;

   if((128+s2)<=N) { s2|=256; }

   return((unsigned char)(s2>>8));
}



