#ifndef __FIR_FILTER_CONFIG_H__
#define __FIR_FILTER_CONFIG_H__

const int32_t FIR_coef[32] = {
			2271,	2540,	2893,	3247,	3719,	4193,	4841,	5495,
    		6422,	7361,	8793,	10244,	12837,	15463,	23719,	32049,
			23719,	15463,	12837,	10244,	8793,	7361,	6422, 	5495,
			4841, 	4193, 	3719, 	3247, 	2893, 	2540, 	2271, 	2001
    };
uint8_t len = 32;
uint32_t final_divider = 262144;

const int32_t FIR_coef2[] = {
		1667,	-2644,	4321,	27166,
        4321,	-2644,	1667,	-1087
   };
uint8_t len2 = 8;
uint32_t final_divider2 = 32768;

#endif
