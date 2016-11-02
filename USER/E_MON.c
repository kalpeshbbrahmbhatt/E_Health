#include "config.h"
#include "E_Mon.h"
#include <stm32f10x.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

void voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL);
void current1(unsigned int _inPinI, double _ICAL);
void calcVI(unsigned int crossings, unsigned int timeout);
double calcIrms(unsigned int Number_of_Samples);
u16 analogRead(u8 ADC_Channel_x);

u8 inPinV;
u8 inPinI1;
u8 inPinI2;
double VCAL;
double ICAL1;
double ICAL2;
double PHASECAL;
int sampleV;  							 //sample_ holds the raw analog read value
int sample1I;    
int sample2I;

double lastFilteredV,filteredV;          //Filtered_ is the raw analog value minus the DC offset
double filtered1I; 
double filtered2I;
double offsetV;                          //Low-pass filter output
double offset1I;                          //Low-pass filter output 
double offset2I;													//Low-pass filter output
double phaseShiftedV;                             //Holds the calibrated phase shifted voltage.
double sqV,sumV,sq1I,sq2I,sum1I,sum2I, inst1P, inst2P, sum1P, sum2P;              //sq = squared, sum = Sum, inst = instantaneous
int startV;                                       //Instantaneous voltage at start of sample window.
bool lastVCross, checkVCross;                  //Used to measure number of times threshold is crossed.
double realPower, apparentPower, powerFactor, Vrms, I1rms, I2rms, Watt1, Watt2;
double rp, ap, pf;



void voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL)
{
   inPinV = _inPinV;
   VCAL = _VCAL;
   PHASECAL = _PHASECAL;
   offsetV = ADC_COUNTS>>1;
}

void current1(unsigned int _inPinI, double _ICAL)
{
   inPinI1 = _inPinI;
   ICAL1 = _ICAL;
   offset1I = ADC_COUNTS>>1;
}

void current2(unsigned int _inPinI, double _ICAL)
{
   inPinI2 = _inPinI;
   ICAL2 = _ICAL;
   offset2I = ADC_COUNTS>>1;
}

void calcVI(unsigned int crossings, unsigned int timeout)
{
	int SupplyVoltage=3300;
	unsigned int crossCount = 0;                             //Used to measure number of times threshold is crossed.
  unsigned int numberOfSamples = 0;                        //This is now incremented  
	unsigned long start; 
	double V_RATIO;
	double I1_RATIO;
	double I2_RATIO;
  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  bool st = false;                                  //an indicator to exit the while loop
	
	start = millis;    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

  while(st == false)                                   //the while loop...
  {
     startV = analogRead(ADC_Channel_0);	//inPinV);                    //using the voltage waveform
     if ((startV < ((float)ADC_COUNTS * 0.55)) && (startV > ((float)ADC_COUNTS * 0.45))) 
			 st = true;  //check its within range
     if ((millis-start)>timeout) 
			 st = true;
  }
  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //------------------------------------------------------------------------------------------------------------------------- 
	start = millis; 

  while ((crossCount < crossings) && ((millis-start)<timeout)) 
  {
    numberOfSamples++;                       //Count number of times looped.
    lastFilteredV = filteredV;               //Used for delay/phase compensation
    
    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
    sampleV = analogRead(ADC_Channel_0);	//inPinV);                 //Read in raw voltage signal
    sample1I = analogRead(ADC_Channel_1);	//inPinI);                 //Read in raw current signal
		sample2I = analogRead(ADC_Channel_4);	//inPinI);                 //Read in raw current signal

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    offsetV = offsetV + ((sampleV-offsetV)/1024); //4096);	
		filteredV = (float)sampleV - offsetV;
    offset1I = offset1I + ((sample1I-offset1I)/1024);	//4096);
		filtered1I = sample1I - offset1I;
		offset2I = offset2I + ((sample2I-offset2I)/1024);	//4096);
		filtered2I = sample2I - offset2I;
   
    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------  
    sqV= filteredV * filteredV;                 //1) square voltage values
    //sumV += sqV;                                //2) sum
		sumV = sumV + sqV;
    
    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------   
    sq1I = filtered1I * filtered1I;                //1) square current values
    sum1I += sq1I;                                //2) sum 
    sq2I = filtered2I * filtered2I;                //1) square current values
    sum2I += sq2I;
    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV); 
    
    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------   
    inst1P = phaseShiftedV * filtered1I;          //Instantaneous Power
    sum1P +=inst1P;                               //Sum 
		inst2P = phaseShiftedV * filtered2I;          //Instantaneous Power
    sum2P +=inst2P;                               //Sum 
    
    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength 
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------       
    lastVCross = checkVCross;                     
    if (sampleV > startV) {
			checkVCross = true; //true; 
		}
    else 
			checkVCross = false;
    if (numberOfSamples==1) 
			lastVCross = checkVCross;                  
                     
    if (lastVCross != checkVCross) 
			crossCount++;
  }
 
  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //------------------------------------------------------------------------------------------------------------------------- 
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied. 
  
  V_RATIO = VCAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples); 
  
  I1_RATIO = ICAL1 *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  I1rms = I1_RATIO * sqrt(sum1I / numberOfSamples);
  I2_RATIO = ICAL2 *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  I2rms = I2_RATIO * sqrt(sum2I / numberOfSamples);	

  //Calculation power values
  realPower = V_RATIO * I1_RATIO * sum1P / numberOfSamples;
  apparentPower = Vrms * I1rms;
  powerFactor=realPower / apparentPower;
	rp = V_RATIO * I2_RATIO * sum2P / numberOfSamples;
	ap = Vrms * I2rms;
	pf = rp / ap;
	Watt1 = Vrms * I1rms * powerFactor;
	Watt2 = Vrms * I2rms * pf;
  //watt = sqrt(3) × PF × amp × volt
  //Reset accumulators
  sumV = 0;
  sum1I = 0;
  sum1P = 0;
	sum2I = 0;
	sum2P = 0;
//-------------------------------------------------------------------------------------- 

}

double calcIrms(unsigned int Number_of_Samples)
{
  
	int SupplyVoltage=3300;
	u16 n;
	double I1_RATIO;
	//double I2_RATIO;
	for (n = 0; n < Number_of_Samples; n++)
  {
    sample1I = analogRead(inPinI1);
		// Digital low pass filter extracts the 2.5 V or 1.65 V dc offset, 
		//  then subtract this - signal is now centered on 0 counts.
    offset1I = (offset1I + (sample1I-offset1I)/1024);		//4096); 
		filtered1I = sample1I - offset1I;
		
    // Root-mean-square method current
    // 1) square current values
    sq1I = filtered1I * filtered1I;
    // 2) sum 
    sum1I += sq1I;
  }

  I1_RATIO = ICAL1 *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  I1rms = I1_RATIO * sqrt(sum1I / Number_of_Samples); 

  //Reset accumulators
  sum1I = 0;
//--------------------------------------------------------------------------------------       
 
  return I1rms;
}


u16 analogRead(u8 ADC_Channel_x)
{
		ADC_RegularChannelConfig(ADC1, ADC_Channel_x , 1, ADC_SampleTime_55Cycles5 );	// Set the conversion channel
		ADC1->CR2|=1<<22;								// Software start ADC 1
		while(!(ADC1->SR & 0x02))				// Wait for the conversion to complete
		{}
		return ADC1->DR;								//Return ADC1 conversion value
}
