/*
Copyright (c) 2022 C Gerrish (https://github.com/Gerriko)
¬ этом примере используетс€ скольз€щее среднее дл€ сглаживани€ выборочных значений емкости.

ќн использует библиотеку емкостного зондировани€ дл€ плат на базе RP2040 (использующих процессор PIO).
≈мкостное зондирование используетс€ дл€ обнаружени€ прикосновени€ на основе электрической

емкости человеческого тела.

Ёта библиотека основана на методах измерени€, используемых в
Capacitive Sensing Library for 'duino / Wiring (метод выборки отличаетс€)
https://github.com/PaulStoffregen/CapacitiveSensor
Copyright (c) 2009 Paul Bagder
Updates for other hardare by Paul Stoffregen, 2010-2016

*/

 #include "PicoCapSensing.h"

// define the pin numbers:
static const int TRIGPIN =  13;
static const int RECPIN =  12;
static const int LEDPIN = 25;

static const uint8_t MOVAVE_CNT = 3;      // Used to calculate a moving ave (needs to be > 0)
static const uint16_t TOUCHSENSETHRESHOLD = 80;

static long totalVal = 0L;
static int totalnow = 0;
static int totalprev = 0;
static int totalsmooth[6];
static uint8_t cntr = 0;
static bool MAcalc = false;

static uint8_t LEDstate = 0;

// Ётот класс предоставит смещение пам€ти PIO дл€ PIO, распознающего капсюль (нужно только один раз)
PicoPIO capPicoPIO(pio0);
PicoCapSensing CapSensor(capPicoPIO, TRIGPIN, RECPIN);

void setup()
{
  // put your setup code here, to run once:
  pinMode(LEDPIN, OUTPUT);

  Serial.begin(115200);
  while(!Serial) {;;}

  // Define the pins for

}


void loop() 
{
  totalnow = CapSensor.getCapSensingSample(2000, 30);
  if (totalnow) 
  {
    if (totalprev) 
    {
      if (cntr < MOVAVE_CNT) 
      {
        if (abs(totalnow - totalprev) < TOUCHSENSETHRESHOLD) {
          if (totalnow > TOUCHSENSETHRESHOLD || totalprev > TOUCHSENSETHRESHOLD) {
            if (totalnow > totalprev) totalsmooth[cntr] = totalnow;
            else  totalsmooth[cntr] = totalprev;
          }
          else totalsmooth[cntr] = abs(totalnow - totalprev);
        }
        else totalsmooth[cntr] = abs(totalnow - totalprev);
        cntr++;
      }
      else {
        cntr = 0;
        if (!MAcalc) MAcalc = true;
      }
      if (MAcalc) {
        totalVal = 0;
        for (uint8_t i = 0; i < MOVAVE_CNT; i++) {
          totalVal += totalsmooth[i];
        }
        totalVal /= MOVAVE_CNT;
        Serial.print("0, ");              // sets the min value on y-axis
        Serial.print(totalVal);           // print smoothed output from capsensor output
        Serial.println(", 2000");           // sets the max value on y-axis (although it can shift)

        if (totalVal > TOUCHSENSETHRESHOLD) 
        {
          if (!LEDstate) 
          {
            LEDstate = !LEDstate;
            digitalWrite(LEDPIN, LEDstate);
          }
        }
        else 
        {
          if (LEDstate) 
          {
            LEDstate = !LEDstate;
            digitalWrite(LEDPIN, LEDstate);
          }
        }
      }
    }
    totalprev = totalnow;
  }
  delay(50);                             // arbitrary delay to limit data to serial port

}
