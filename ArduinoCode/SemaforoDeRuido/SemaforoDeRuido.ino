//Credits:  https://blog.yavilevich.com/2016/08/arduino-sound-level-meter-and-spectrum-analyzer/

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif


// pins
#define MicPin A0 // used with analogRead mode only
#define LedPin 6 

// consts
#define AmpMax (1024/2)
#define MicSamples (1024*2) // Three of these time-weightings have been internationally standardised, 'S' (1 s) originally called Slow, 'F' (125 ms) originally called Fast and 'I' (35 ms) originally called Impulse.

// modes
#define Use3.3 // use 3.3 voltage. the 5v voltage from usb is not regulated. this is much more stable.
//#define ADCReClock // switch to higher clock, not needed if we are ok with freq between 0 and 4Khz.
#define ADCFlow // read data from adc with free-run (not interupt). much better data, dc low. hardcoded for A0.

#define FreqLog // use log scale for FHT frequencies
#ifdef FreqLog
#define FreqOutData fht_log_out
#define FreqGainFactorBits 0
#else
#define FreqOutData fht_lin_out8
#define FreqGainFactorBits 3
#endif
#define FreqSerialBinary

#define VolumeGainFactorBits 0

#define DbCalibration 94

#define awgsize 5

// macros
// http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

int prevdb = 0;

const int maxScale = 12;
const int redZone = 8;
const int orangeZone = 5;

Adafruit_NeoPixel pixels(maxScale, LedPin, NEO_GRB + NEO_KHZ800);

const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;


void setup() 
{
	//pinMode(MicPin, INPUT); // relevant for digital pins. not relevant for analog. however, don't put into digital OUTPUT mode if going to read analog values.

#ifdef ADCFlow
	// set the adc to free running mode
	// register explanation: http://maxembedded.com/2011/06/the-adc-of-the-avr/
	// 5 => div 32. sample rate 38.4
	// 7 => switch to divider=128, default 9.6khz sampling
	ADCSRA = 0xe0+7; // "ADC Enable", "ADC Start Conversion", "ADC Auto Trigger Enable" and divider.
	ADMUX = 0x0; // use adc0 (hardcoded, doesn't use MicPin). Use ARef pin for analog reference (same as analogReference(EXTERNAL)).
#ifndef Use3.3
	ADMUX |= 0x40; // Use Vcc for analog reference.
#endif
	DIDR0 = 0x01; // turn off the digital input for adc0
#else
#ifdef Use3.3
	analogReference(EXTERNAL); // 3.3V to AREF
#endif
#endif

#ifdef ADCReClock // change ADC freq divider. default is div 128 9.6khz (bits 111)
	// http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
	// 1 0 0 = mode 4 = divider 16 = 76.8khz
	//sbi(ADCSRA, ADPS2);
	//cbi(ADCSRA, ADPS1);
	//cbi(ADCSRA, ADPS0);
	// 1 0 1 = mode 5 = divider 32 = 38.4Khz
	sbi(ADCSRA, ADPS2);
	cbi(ADCSRA, ADPS1);
	sbi(ADCSRA, ADPS0);
#endif


   Serial.begin(9600);
    pixels.begin();

animation(10);



}

int averagegdb(int prev, int next){
  return (next * (1/awgsize) + prev * (1-(1/awgsize)))/2 ; 
}



  int previousValue = 0;
void loop() 
{



   int db = MeasureVolume();

   int avgdb = averagegdb(prevdb, db);
    prevdb = db;
	Serial.print(db);
	Serial.print(",");
  Serial.println(avgdb);


   // map 1v p-p level to the max scale of the display
   int displayPeak = map(db, 40, 90, 0, maxScale);


  
   // draw the new sample
   for (int i = 0; i <= maxScale; i++)
   {
      if ( !((i == displayPeak | i == previousValue) | ( displayPeak > previousValue  & i < displayPeak & i > previousValue ) | ( displayPeak < previousValue  & i > displayPeak & i < previousValue )   ) )
      {
        pixels.setPixelColor(i, pixels.Color(0, 2, 2));
      }
      else if (i >= orangeZone & i < redZone) // draw in green
      {
        pixels.setPixelColor(i, pixels.Color(125, 125, 0));
        if(i != displayPeak) pixels.setPixelColor(i, pixels.Color(3, 3, 0));
      }
      else if (i < redZone) // draw in green
      {
        pixels.setPixelColor(i, pixels.Color(0, 250, 0));
        if(i != displayPeak) pixels.setPixelColor(i, pixels.Color(0, 8, 0));
      }
      else // Red Alert!  Red Alert!
      {
        pixels.setPixelColor(i, pixels.Color(250, 0, 0));
        if(i != displayPeak) pixels.setPixelColor(i, pixels.Color(8, 0, 0));
      }
   }
   pixels.show();;  // write the changes we just made to the strip
   previousValue = displayPeak;
}






void animation(int iteraciones){
  // These are the pixels in order of animation-- 36 pixels in total:
int sine[] = {
   0,1,2,3,4,5,6,7,8,9,10,11 };
uint32_t color = pixels.Color(250, 14, 14); 
for (int j=0; j<10; j++){
for(int i=0; i<12; i++) {
    pixels.setPixelColor(sine[i], color);             // Draw 'head' pixel
    pixels.setPixelColor(sine[(i + 12 - 2) % 12], 0); // Erase 'tail'
    pixels.show();
    delay(40);
  }}}


  // calculate volume level of the signal and print to serial and LCD
float MeasureVolume()
{
	long soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0, t0 = millis();
	//cli();  // UDRE interrupt slows this way down on arduino1.0
	for (int i = 0; i < MicSamples; i++)
	{
#ifdef ADCFlow
		while (!(ADCSRA & /*0x10*/_BV(ADIF))); // wait for adc to be ready (ADIF)
		sbi(ADCSRA, ADIF); // restart adc
		byte m = ADCL; // fetch adc data
		byte j = ADCH;
		int k = ((int)j << 8) | m; // form into an int
#else
		int k = analogRead(MicPin);
#endif
		int amp = abs(k - AmpMax);
		amp <<= VolumeGainFactorBits;
		soundVolMax = max(soundVolMax, amp);
		soundVolAvg += amp;
		soundVolRMS += ((long)amp*amp);
	}
	soundVolAvg /= MicSamples;
	soundVolRMS /= MicSamples;
	float soundVolRMSflt = sqrt(soundVolRMS);
	//sei();

	float dB = 20.0*log10(soundVolRMSflt/AmpMax)+DbCalibration;

	// convert from 0 to 100
	soundVolAvg = 100 * soundVolAvg / AmpMax; 
	soundVolMax = 100 * soundVolMax / AmpMax; 
	soundVolRMSflt = 100 * soundVolRMSflt / AmpMax;
	soundVolRMS = 10 * soundVolRMSflt / 7; // RMS to estimate peak (RMS is 0.7 of the peak in sin)

	// print
	//Serial.print("Time: " + String(millis() - t0));
	//Serial.print(" Amp: Max: " + String(soundVolMax));
	//Serial.print("% Avg: " + String(soundVolAvg));

  return dB;

}