#ifndef THERMISTORTABLES_H_
#define THERMISTORTABLES_H_

#include "Marlin.h"

#define OVERSAMPLENR 4

#if (THERMISTORHEATER_0 == 1) || (THERMISTORBED == 1) //100k bed thermistor

// termistor table for semitec 104gt-2 used by E3D v6 hotend
const float temptable_1[61][2] ={
{	107.16	,	300	},
{	114.83	,	295	},
{	123.19	,	290	},
{	132.31	,	285	},
{	142.28	,	280	},
{	153.19	,	275	},
{	165.16	,	270	},
{	178.29	,	265	},
{	192.73	,	260	},
{	208.62	,	255	},
{	226.15	,	250	},
{	245.51	,	245	},
{	266.93	,	240	},
{	290.67	,	235	},
{	317.01	,	230	},
{	346.31	,	225	},
{	378.95	,	220	},
{	415.37	,	215	},
{	456.11	,	210	},
{	501.75	,	205	},
{	552.99	,	200	},
{	610.64	,	195	},
{	675.64	,	190	},
{	749.07	,	185	},
{	832.24	,	180	},
{	926.64	,	175	},
{	1034.1	,	170	},
{	1156.6	,	165	},
{	1296.7	,	160	},
{	1457.3	,	155	},
{	1641.9	,	150	},
{	1854.8	,	145	},
{	2100.8	,	140	},
{	2386.1	,	135	},
{	2717.9	,	130	},
{	3105	,	125	},
{	3558.1	,	120	},
{	4090.1	,	115	},
{	4717	,	110	},
{	5458.4	,	105	},
{	6338.3	,	100	},
{	7386.7	,	95	},
{	8640.7	,	90	},
{	10147	,	85	},
{	11963	,	80	},
{	14164	,	75	},
{	16841	,	70	},
{	20114	,	65	},
{	24136	,	60	},
{	29100	,	55	},
{	35262	,	50	},
{	42950	,	45	},
{	52598	,	40	},
{	64776	,	35	},
{	80239	,	30	},
{	100000	,	25	},
{	125420	,	20	},
{	158340	,	15	},
{	201270	,	10	},
{	257690	,	5	},
{	332400	,	0	}
};
#endif

#define _TT_NAME(_N) temptable_ ## _N
#define TT_NAME(_N) _TT_NAME(_N)

#ifdef THERMISTORHEATER_0
# define HEATER_0_TEMPTABLE TT_NAME(THERMISTORHEATER_0)
# define HEATER_0_TEMPTABLE_LEN (sizeof(HEATER_0_TEMPTABLE)/sizeof(*HEATER_0_TEMPTABLE))
#else
# ifdef HEATER_0_USES_THERMISTOR
#  error No heater 0 thermistor table specified
# else  // HEATER_0_USES_THERMISTOR
#  define HEATER_0_TEMPTABLE NULL
#  define HEATER_0_TEMPTABLE_LEN 0
# endif // HEATER_0_USES_THERMISTOR
#endif

//Set the high and low raw values for the heater, this indicates which raw value is a high or low temperature
#ifndef HEATER_0_RAW_HI_TEMP
# ifdef HEATER_0_USES_THERMISTOR   //In case of a thermistor the highest temperature results in the lowest ADC value
#  define HEATER_0_RAW_HI_TEMP 0
#  define HEATER_0_RAW_LO_TEMP 16383
# else                          //In case of an thermocouple the highest temperature results in the highest ADC value
#  define HEATER_0_RAW_HI_TEMP 16383
#  define HEATER_0_RAW_LO_TEMP 0
# endif
#endif

#ifdef THERMISTORBED
# define BEDTEMPTABLE TT_NAME(THERMISTORBED)
# define BEDTEMPTABLE_LEN (sizeof(BEDTEMPTABLE)/sizeof(*BEDTEMPTABLE))
#else
# ifdef BED_USES_THERMISTOR
#  error No bed thermistor table specified
# endif // BED_USES_THERMISTOR
#endif

//Set the high and low raw values for the heater, this indicates which raw value is a high or low temperature
#ifndef HEATER_BED_RAW_HI_TEMP
# ifdef BED_USES_THERMISTOR   //In case of a thermistor the highest temperature results in the lowest ADC value
#  define HEATER_BED_RAW_HI_TEMP 0
#  define HEATER_BED_RAW_LO_TEMP 16383
# else                          //In case of an thermocouple the highest temperature results in the highest ADC value
#  define HEATER_BED_RAW_HI_TEMP 16383
#  define HEATER_BED_RAW_LO_TEMP 0
# endif
#endif

#endif //THERMISTORTABLES_H_
