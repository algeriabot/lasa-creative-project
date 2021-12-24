/*
 * 
 * LASA Creative Project
 * Gabe Tao
 * Hey, thanks for taking a look at my project!
 * This is the code that was uploaded to my board inside the guitar.
 * Quick disclaimer: some of this code was not written by me. This includes small chunks that process frequencies, as it is way too 
 * advanced for an 8th grade student to understand. Treat it like an external library.
 * I assure you that all the other code was written wholly by me and that I poured hard work into the hardware and software design.
 * 
 */



//Import libraries
#include <pitches.h>
#include <Adafruit_NeoPixel.h>
#include "IRremote.h"


//--------------------------STARTUP DEFINITIONS------------------------------//
int  in[128];
byte NoteV[13] = {8, 23, 40, 57, 76, 96, 116, 138, 162, 187, 213, 241, 255};
float f_peaks[8]; // top 8 frequencies peaks in descending order
int last_chord = 0;
#define LED_PIN 6
#define LED_COUNT 35
#define NOISE 1
#define TONE_PIN 5
Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip(8, 11, NEO_GRB + NEO_KHZ800);
IRrecv irrecv(2);    
decode_results results;   
int lvl = 10;
int mode = 0;
bool detected = false;
int tunerMode = 1;
//---------------------------------------------------------------------------//








//----------------------SETUP FUNCTION---------------------------------------//
void setup() {

  ring.begin();
  strip.begin();
  ring.show();
  strip.show();
  colorFade(7,90,7);
  Serial.begin(9600);
  irrecv.enableIRIn();    
  Serial.println(F("start")); 

}
//---------------------------------------------------------------------------//









//----------------------HELPER FUNCTIONS-------------------------------------//

bool IR_idle() {
  return irrecv.decode(&results) || results.rawlen == 0;
}

void colorFade(int endR, int endG, int endB) {

  uint8_t startR, startG, startB;
  uint32_t startColor = strip.getPixelColor(0); // get the current color
  startB = startColor & 0xFF;
  startG = (startColor >> 8) & 0xFF;
  startR = (startColor >> 16) & 0xFF;

  //Repeat until all values have been moved to the target
  while ((startR != endR) || (startG != endG) || (startB != endB)) {

    if (startR < endR) startR++; else if (startR > endR) startR--;  // increment or decrement the old color values
    if (startG < endG) startG++; else if (startG > endG) startG--;
    if (startB < endB) startB++; else if (startB > endB) startB--;

    ring.fill(ring.Color(startR, startG, startB));
    ring.show();
    strip.fill(strip.Color(startR, startG, startB));
    strip.show();

    delay(1);
    
  }
}



uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return ring.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return ring.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return ring.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}


//---------------------------------------------------------------------------//





//--------------------------CHORD DETECTION----------------------------------//

void chordDetection() {
  long unsigned int startTime, endTime, a2;
  float a;
  float sum1 = 0, sum2 = 0;
  float sampling;
  uint32_t color;
  startTime = micros();
  //Take 128 samples of data
  for (int i = 0; i < 128; i++)
  {
    a = analogRead(A0) - 500; //Take average of two mics and do a rough zero shift
    //utilising time between two sample for windowing & amplitude calculation
    sum1 = sum1 + a;          //to average value
    sum2 = sum2 + a * a;      // to RMS value
    a = a * (sin(i * 3.14 / 128) * sin(i * 3.14 / 128)); // Hann window
    in[i] = 4 * a;            // scaling for float to int conversion
    delayMicroseconds(195);   // based on operation frequency range
  }
  endTime = micros();
  sum1 = sum1 / 128;           //average amplitude
  sum2 = sqrt(sum2 / 128);     //RMS amplitude
  sampling = 128000000 / (endTime - startTime); // real time sampling frequency; 128 million

  //Threshold of very small amplitudes (3). Only run if above this value.
  if (sum2 - sum1 > 18) {
    FFT(128, sampling);       //EasyFFT based optimised  FFT code, updates f_peaks list (not written by me)

    //reset in list
    for (int i = 0; i < 12; i++) {
      in[i] = 0;
    }

    int j = 0, k = 0; //below loop will convert frequency value to note
    for (int i = 0; i < 8; i++)
    {
      if (f_peaks[i] > 1040) {
        f_peaks[i] = 0;
      }
      if (f_peaks[i] >= 65.4   && f_peaks[i] <= 130.8) {
        f_peaks[i] = 255 * ((f_peaks[i] / 65.4) - 1);
      }
      if (f_peaks[i] >= 130.8  && f_peaks[i] <= 261.6) {
        f_peaks[i] = 255 * ((f_peaks[i] / 130.8) - 1);
      }
      if (f_peaks[i] >= 261.6  && f_peaks[i] <= 523.25) {
        f_peaks[i] = 255 * ((f_peaks[i] / 261.6) - 1);
      }
      if (f_peaks[i] >= 523.25 && f_peaks[i] <= 1046)  {
        f_peaks[i] = 255 * ((f_peaks[i] / 523.25) - 1);
      }
      if (f_peaks[i] >= 1046 && f_peaks[i] <= 2093)  {
        f_peaks[i] = 255 * ((f_peaks[i] / 1046) - 1);
      }
      if (f_peaks[i] > 255) {
        f_peaks[i] = 254;
      }
      j = 1; k = 0;
      while (j == 1)
      {
        if (f_peaks[i] <= NoteV[k]) {
          f_peaks[i] = k;
          j = 0;
        }
        k++;  // a note with max peaks (harmonic) with aplitude priority is selected
        if (k > 15) {
          j = 0;
        }
      }

      if (f_peaks[i] == 12) {
        f_peaks[i] = 0;
      }
      k = f_peaks[i];
      in[k] = in[k] + (8 - i);
    }

    k = 0; j = 0;
    for (int i = 0; i < 12; i++)
    {
      if (k < in[i]) {
        k = in[i];  //Max value detection
        j = i;
      }
    }

    for (int i = 0; i < 8; i++)
    {
      in[12 + i] = in[i];
    }

    for (int i = 0; i < 12; i++)
    {
      in[20 + i] = in[i] * in[i + 4] * in[i + 7];
      in[32 + i] = in[i] * in[i + 3] * in[i + 7]; //all chord check
    }


    for (int i = 0; i < 24; i++)
    {
      in[i] = in[i + 20];
      if (k < in[i]) {
        k = in[i];  // picking chord with max possiblity
        j = i;
      }
    }
    char chord_out;
    int chord = j;
    if (chord > 11) {
      chord = chord - 12;  //Major-minor check
      chord_out = 'm';
    } else {
      chord_out = 'M';
    }

    //Push data to ring
    //ring.show();

    // Here "chord" variable has value of detected chord,
    // 0-11 defines all major chord from C,C#,D,D#,.. B
    //12-23 defines all minor chord from Cm,C#m,Dm,D#m,.. Bm





    


    a2 = micros();
    //Serial.println(IR_idle());

    //Run if the same chord is detected twice in a row
    if ((last_chord == chord) && (IR_idle())) {
      //Change color based on note of chord
      if (chord == 0) {
        colorFade(125, 255, 0);
      }
      if (chord == 1) {
        colorFade(0, 255, 0);
      }
      if (chord == 2) {
        colorFade(0, 255, 34);
      }
      if (chord == 3) {
        colorFade(0, 255, 255);
      }
      if (chord == 4) {
        colorFade(0, 125, 255);
      }
      if (chord == 5) {
        colorFade(0, 0, 255);
      }
      if (chord == 6) {
        colorFade(125, 0, 255);
      }
      if (chord == 7) {
        colorFade(255, 0, 255);
      }
      if (chord == 8) {
        colorFade(255, 0, 125);
      }
      if (chord == 9) {
        colorFade(255, 0, 0);
      }
      if (chord == 10) {
        colorFade(255, 34, 0);
      }
      if (chord == 11) {
        colorFade(255, 255, 0);
      }

    }

    //Set the last chord
    last_chord = chord;


  }
}

//---------------------------------------------------------------------------//









//--------------------------VOLUME DETECTION--------------------------------//

void volumeDetection() {
  long n = 0;
    long sum = 0;
    int pixels = 0;
    int pixels_strip = 0;
    for(int i=0; i<10; i++)
    {
       sum += analogRead(A0);
       //sum += analogRead(A4);
    }

    n = sum/10 - 512;
    n  = (n <= NOISE) ? 0 : (n - NOISE);
    lvl = ((lvl * 7) + n) >> 3;
    //Serial.println(lvl);


    //Make sure the number of pixels can't go over
    if (lvl > 36) {
      lvl = 36;
    }

    if (lvl == 0) {
      lvl = 1;
    }


    if (IR_idle()) {


                      //Reset ring
                      ring.clear();
                  
                  
                      //Adjust volume levels: at measurement 36 it will map to full pixels.
                      pixels = map(lvl, 0, 36, 0, 35);
                      //Serial.println(pixels);
                  
                  
                      //ring.fill(ring.Color(12,12,12));
                  
                      //Loop through all the pixels to light and use Wheel to generate color wheel value
                      for (int i=0; i<pixels; i++) {
                        ring.setPixelColor(i, ring.gamma32(Wheel(map(i, 0, ring.numPixels()-1, 0, 255))));
                      }
                  
                      //Show ring data
                      ring.show();
                  
                      strip.clear();
                  
                      pixels_strip = map(lvl, 0, 36, 0, 8);
                      
                      //Loop through all the pixels to light and use Wheel to generate color wheel value
                      for (int i=0; i<pixels_strip; i++) {
                        strip.setPixelColor(i, strip.gamma32(Wheel(map(i, 0, strip.numPixels()-1, 0, 255))));
                      }
                  
                      //Show ring data
                      strip.show();


    } else {
      //Serial.println("sk");
    }
}

//---------------------------------------------------------------------------//






//--------------------------REMOTE CONTROL FUNCTION--------------------------//

void remoteControl(long data) {

  
  if (data == 16195807) {
    //red
    colorFade(255,0,0);
  } else if (data == 16228447) {
    //green
    colorFade(0,255,0);
  } else if (data == 16212127) {
    //blue
    colorFade(0,0,255);
  } else if (data == 16244767) {
    //white
    colorFade(150,150,150);
  } else if (data == 16191727) {
    //light red
    colorFade(100,7,7);
  } else if (data == 16224367) {
    //light green
    colorFade(7,100,7);
  } else if (data == 16208047) {
    //light blue
    colorFade(7,7,100);
  } else if (data == 16199887) {
    //orange
    colorFade(255, 34, 0);
  } else if (data == 16232527) {
    //sky blue
    colorFade(135,206,235);
  } else if (data == 16216207) {
    //dark purple
    colorFade(34, 0, 34);
  } else if (data == 16189687) {
    //light orange
    colorFade(255, 100, 0);
  } else if (data == 16222327) {
    //light teal
    colorFade(0,100,100);
  } else if (data == 16206007) {
    //light purple
    colorFade(100,0,100);
  } else if (data == 16197847) {
    //yellow
    colorFade(255,255,0);
  } else if (data == 16230487) {
    //dark teal
    colorFade(0,50,50);
  } else if (data == 16214167) {
    //magenta
    colorFade(255,0,255);
  } else if (data == 16203967) {
    //OFF
    colorFade(0,0,0);
  }
}

//---------------------------------------------------------------------------//










//--------------------------TUNER FUNCTION--------------------------//

void tuner(int stringNum) {
  
  switch (stringNum) {
    case 0:
      noTone(TONE_PIN);
      break;
    case 1:
      tone(TONE_PIN, NOTE_E5);
      break;
    case 2:
      tone(TONE_PIN, NOTE_B4);
      break;
    case 3:
      tone(TONE_PIN, NOTE_G4);
      break;
    case 4:
      tone(TONE_PIN, NOTE_D4);
      break;
    case 5:
      tone(TONE_PIN, NOTE_A3);
      break;
    case 6:
      tone(TONE_PIN, NOTE_E3);
      break;
  }
    
}

//---------------------------------------------------------------------------//









//-------------------------------MAIN LOOP-----------------------------------//

void loop()
{

  detected = false;
  if (irrecv.decode(&results)) {
    detected = true;
    // Returns 0 if no data ready, 1 if data ready.
    Serial.println(" ");     
    Serial.print(F("Code: "));
    Serial.print(results.value, HEX);
    Serial.print(" ");
    Serial.println(results.value); //prints the value on a button press     
    Serial.println(" ");


    //Mode 1: chord mode
    //Mode 2: volume mode
    //Mode 3: tuner mode
    //Mode 4: remote mode
    

    if (results.value == 16240687) {
      colorFade(0,0,0);
      mode = 1;
      Serial.println("m1");
    }
    else if (results.value == 16248847) {
      Serial.println("m2");
      colorFade(0,0,0);
      mode = 2;
    }
    else if (results.value == 16238647) {
      Serial.println("m3");
      mode = 3;
      colorFade(0,0,0);
      tuner(tunerMode);
      tunerMode += 1;
      if (tunerMode == 7) {
        tunerMode = 0;
      }
    }
    else if (results.value == 16246807) {
      Serial.println("m4");
      mode = 4;
      colorFade(0,0,0);
    }

    irrecv.resume();

     
        
  }

  
  switch (mode) {
    case 1:
      //Serial.println("m1");
      noTone(TONE_PIN);
      chordDetection();
      break;

    case 2:
      //Serial.println("m2");
      noTone(TONE_PIN);
      volumeDetection();
      break;

    case 3: break;


    case 4: 
  
      if (detected) {
        noTone(TONE_PIN);
        remoteControl(results.value);
      }
  }
  

}
//---------------------------------------------------------------------------//
















































//-----------------------------FFT Function----------------------------------------------//
// Advanced compressed math code library (not written/edited by me), does Fast Fourier Transform on raw wave data
// EasyFFT code optimised for 128 sample size to reduce mamory consumtion

float FFT(byte N, float Frequency)
{
  byte data[8] = {1, 2, 4, 8, 16, 32, 64, 128};
  int a, c1, f, o, x;
  a = N;

  for (int i = 0; i < 8; i++)          //calculating the levels
  {
    if (data[i] <= a) {
      o = i;
    }
  }
  o = 7;
  byte in_ps[data[o]] = {};   //input for sequencing
  float out_r[data[o]] = {}; //real part of transform
  float out_im[data[o]] = {}; //imaginory part of transform

  x = 0;
  for (int b = 0; b < o; b++)              // bit reversal
  {
    c1 = data[b];
    f = data[o] / (c1 + c1);
    for (int j = 0; j < c1; j++)
    {
      x = x + 1;
      in_ps[x] = in_ps[j] + f;
    }
  }

  for (int i = 0; i < data[o]; i++)     // update input array as per bit reverse order
  {
    if (in_ps[i] < a)
    {
      out_r[i] = in[in_ps[i]];
    }
    if (in_ps[i] > a)
    {
      out_r[i] = in[in_ps[i] - a];
    }
  }


  int i10, i11, n1;
  float e, c, s, tr, ti;

  for (int i = 0; i < o; i++)                             //fft
  {
    i10 = data[i];            // overall values of sine cosine
    i11 = data[o] / data[i + 1]; // loop with similar sine cosine
    e = 6.283 / data[i + 1];
    e = 0 - e;
    n1 = 0;

    for (int j = 0; j < i10; j++)
    {
      c = cos(e * j);
      s = sin(e * j);
      n1 = j;

      for (int k = 0; k < i11; k++)
      {
        tr = c * out_r[i10 + n1] - s * out_im[i10 + n1];
        ti = s * out_r[i10 + n1] + c * out_im[i10 + n1];

        out_r[n1 + i10] = out_r[n1] - tr;
        out_r[n1] = out_r[n1] + tr;

        out_im[n1 + i10] = out_im[n1] - ti;
        out_im[n1] = out_im[n1] + ti;

        n1 = n1 + i10 + i10;
      }
    }
  }


  //---> here onward out_r contains amplitude and our_in conntains frequency (Hz)
  for (int i = 0; i < data[o - 1]; i++)      // getting amplitude from compex number
  {
    out_r[i] = sqrt((out_r[i] * out_r[i]) + (out_im[i] * out_im[i])); // to  increase the speed delete sqrt
    out_im[i] = (i * Frequency) / data[o];
    /*
      Serial.print(out_im[i],2); Serial.print("Hz");
      Serial.print("\t");                            // uncomment to print freuency bin
      Serial.println(out_r[i]);
    */
  }

  x = 0;     // peak detection
  for (int i = 1; i < data[o - 1] - 1; i++)
  {
    if (out_r[i] > out_r[i - 1] && out_r[i] > out_r[i + 1])
    { in_ps[x] = i;  //in_ps array used for storage of peak number
      x = x + 1;
    }
  }

  s = 0;
  c = 0;
  for (int i = 0; i < x; i++)      // re arraange as per magnitude
  {
    for (int j = c; j < x; j++)
    {
      if (out_r[in_ps[i]] < out_r[in_ps[j]])
      { s = in_ps[i];
        in_ps[i] = in_ps[j];
        in_ps[j] = s;
      }
    }
    c = c + 1;
  }

  for (int i = 0; i < 8; i++) // updating f_peak array (global variable)with descending order
  {
    // f_peaks[i]=out_im[in_ps[i]];Serial.println(f_peaks[i]);
    f_peaks[i] = (out_im[in_ps[i] - 1] * out_r[in_ps[i] - 1] + out_im[in_ps[i]] * out_r[in_ps[i]] + out_im[in_ps[i] + 1] * out_r[in_ps[i] + 1])
                 / (out_r[in_ps[i] - 1] + out_r[in_ps[i]] + out_r[in_ps[i] + 1]);
    // Serial.println(f_peaks[i]);
  }
}

//------------------------------------------------------------------------------------//
