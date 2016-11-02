#include "stm32f10x.h"
#include "config.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>


#ifndef READVCC_CALIBRATION_CONST
#define READVCC_CALIBRATION_CONST 1126400L
#endif

#define ADC_BITS    12

#define ADC_COUNTS  (1<<ADC_BITS)

extern void voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL);
extern void current1(unsigned int _inPinI, double _ICAL);
extern void current2(unsigned int _inPinI, double _ICAL);
extern void calcVI(unsigned int crossings, unsigned int timeout);
double calcIrms(unsigned int Number_of_Samples);
extern u16 analogRead(u8 ADC_Channel_x);

extern u8 inPinV;
extern u8 inPinI;
extern double VCAL;
extern double ICAL1;
extern double ICAL2;
extern double PHASECAL;
extern int sampleV;  							 //sample_ holds the raw analog read value
extern int sample1I;  
extern int sample2I;

extern double lastFilteredV,filteredV;          //Filtered_ is the raw analog value minus the DC offset
extern double filtered1I; 
extern double filtered2I;
extern double offsetV;                          //Low-pass filter output
extern double offset1I;                          //Low-pass filter output               
extern double phaseShiftedV;                             //Holds the calibrated phase shifted voltage.
extern double sqV,sumV,sq1I,sq2I,sum1I,inst1P,inst2P,sum1P,sum2P;              //sq = squared, sum = Sum, inst = instantaneous
extern int startV;                                       //Instantaneous voltage at start of sample window.
//extern bool lastVCross, checkVCross;                  //Used to measure number of times threshold is crossed.
extern double realPower, apparentPower, powerFactor, Vrms, I1rms, I2rms, Watt1, Watt2;
extern double rp, ap, pf;
