/*
   This is the Arduino Code for the HobbyPCB RS-HFIQ Transceiver
   It's designed to run on an Arduino Nano which is plugged into the RS-HFIQ

   Completely open source, hack/modify/share as needed but don't complain about it

   Uses NT7S's Si5351Arduino Library which you can find here:
    https://github.com/etherkit/Si5351Arduino
*/

#include "RSHFIQ.h"
#include "si5351.h"
#include "Wire.h"
#include "Arduino.h"
#include <stdint.h>
#include <SendOnlySoftwareSerial.h>
#include <EEPROM.h>

#define Version_Number "RS-HFIQ FW 3.0b"

// The Silicon Labs SI-5351 chip is used to generate RF signals
// for the RS-HFIQ. It generates the LO signal for the up/down converter,
// the Built-in Test (BIT) signal and a third signal which is either
// sent to the 'EXT RF' jack or used to generate CW
Si5351 freq_gen;

// This sets up a send only serial port on EX_GP to send
// frequency data to the Hardrock-50 power amplifier
SendOnlySoftwareSerial HR50 (EX_GP);

long F_Offset;

long TEMP_C, T_TOT = 0, OTEMP;
long T_ARY[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned char T_NDX = 0;
unsigned char Ser_Flag = 0, Ser_NDX = 0;
char S_Input[16], O_DIT = 1, O_DAH = 1;
uint32_t LO_freq, BIT_freq, EXT_freq;
uint64_t F_Long;
byte band = 0, A_29 = 0;
int clip;
byte clip_on = 0;
int cl[] = {285, 340, 250, 250, 330, 330, 430, 300, 308};
byte HR50band, oldHRB = 0;
byte T_count = 0;

// Setup code runs once before the loop starts to initialize
// the board components
void setup() {

  Serial.begin(57600); // Communicate via USB serial port at 57600 Baud
  HR50.begin(19200); // Communicate with the HR50 at 19200 Baud

  // Set up the band select and keying lines
  // and set them all low
  pinMode(B80M, OUTPUT); digitalWrite (B80M, LOW);
  pinMode(B60_40M, OUTPUT); digitalWrite (B60_40M, LOW);
  pinMode(B30_20M, OUTPUT); digitalWrite (B30_20M, LOW);
  pinMode(B17_15M, OUTPUT); digitalWrite (B17_15M, LOW);
  pinMode(B12_10M, OUTPUT); digitalWrite (B12_10M, LOW);
  pinMode(RX_HI, OUTPUT); digitalWrite (RX_HI, HIGH);
  pinMode(TX_HI, OUTPUT); digitalWrite (TX_HI, LOW);
  pinMode(DC_PTT, OUTPUT); digitalWrite (DC_PTT, LOW);
  pinMode(EX_PTT, OUTPUT); digitalWrite (EX_PTT, LOW);
  pinMode(R_LED, OUTPUT); digitalWrite (R_LED, LOW);
  pinMode(Y_LED, OUTPUT); digitalWrite (Y_LED, LOW);
  pinMode(DIT_PD, INPUT);
  pinMode(DAH_PD, INPUT);

  // Analog A0 is used to read the temperature of the crystal and adjust
  // the frequency accordingly. There is a 2.048V precision voltage reference on
  // The AREF pin so we'll use it as our reference
  analogReference(EXTERNAL);

  // Let's measure the temperature 32 times to fill up the average array
  for (byte i = 0; i < 32; i++) Calc_Temp();
  OTEMP = TEMP_C;

 
  // The fixed frequency offset is also stored in EEPROM. We'll restore it and of the value isn't
  // between -10000 and +10000 then we'll reset it to 0
  F_Offset = readLong(1);
  if (abs(F_Offset) > 10000) {
    F_Offset = 0;
    writeLong(1, 0.0);
  }

  // Set the HR50's band to 'UNK' so it doesn't key up before the frequency is set
  HR50.println ("HRBN99;");

  // Set up the SI5351 for a TCXO, set PLLA for
  // 900 MHZ and turn off all outputs
  freq_gen.init(SI5351_CRYSTAL_LOAD_0PF, 0, 0);
  freq_gen.set_ms_source(SI5351_CLK0, SI5351_PLLA);
  freq_gen.set_ms_source(SI5351_CLK1, SI5351_PLLB);
  freq_gen.set_ms_source(SI5351_CLK2, SI5351_PLLB);
  freq_gen.set_pll(89999900000ULL, SI5351_PLLA);
}


void loop() {
  char c;

  // Temperature routine - Every 50 times through the loop it calculates the average temperature based on the last 32 samples
  // of the temperature sensor. 
  if (T_count++ == 50){ 
    T_count = 0;              // reset the counter
    Calc_Temp();              // calculate the average temperature
  }


  // Process incoming serial data
  while (Serial.available() > 0) {   // Process any and all characters in the buffer
    c = Serial.read();               // Get a character
    if (c == '*') {                  // No matter where we are in the state machine, a '*' say clear the buffer and start over.
      Ser_NDX = 0;                   // character index = 0
      Ser_Flag = 1;                  // State 1 means the '*' has been received and we are collecting characters for processing
      for (int z = 0; z < 16; z++) { // Fill the buffer with spaces
        S_Input[z] = ' ';
      }
    }
    else {
      if (Ser_Flag == 1 && c != 13) S_Input[Ser_NDX++] = c;  // If we are in state 1 and the character isn't a <CR>, put it in the buffer
      if (Ser_Flag == 1 && c == 13) {                        // If it is a <CR> ...
        S_Input[Ser_NDX] = 0;                                // terminate the input string with a null (0)
        Ser_Flag = 3;                                        // Set state to 3 to indicate a command is ready for processing
      }
      if (Ser_NDX > 15) Ser_NDX = 15;   // If we've received more than 15 characters without a <CR> just keep overwriting the last character
    }
  }


  // If a complete command is received, process it
  if (Ser_Flag == 3) {
    Command_P();
    Ser_Flag = 0;
  }

  c = digitalRead(DAH_PD);
  if (O_DAH != c) {
    O_DAH = c;
    if (c == 0) TX_ON();
    else TX_OFF();
  }

  if (analogRead(1) > clip) {
    digitalWrite(R_LED, HIGH);
    clip_on = 1;
  }
  else {
    digitalWrite(R_LED, LOW);
    clip_on = 0;
  }

}

void Calc_Temp(void) {
  long T_CALC;

  T_CALC = analogRead(TS_PIN) * 2000UL;
  T_ARY[T_NDX] = (T_CALC - 400000) / 195;

  T_TOT += T_ARY[T_NDX];
  if (++T_NDX > 31) T_NDX = 0;
  TEMP_C = T_TOT / 32;
  T_TOT -= T_ARY[T_NDX];
}

uint64_t F_Adjust(long FREQ) {
  float corr = float(FREQ) / 1000000.0 * float(F_Offset);
  return uint64_t(corr);
}

void Command_P(void) {
  int n;
  char c;
  si5351_clock clock_sel;
  si5351_drive drive;

  switch (S_Input[0]) {
    case '?':
      Serial.println("RSHFIQ");
      break;
    
    case 'B': case 'b':
      if (S_Input[1] == '?') {
        if (BIT_freq < 10000000) Serial.print("0");
        Serial.println(BIT_freq);
      }
      else {
        BIT_freq = Conv_Freq();
        if (BIT_freq > 1000000 && BIT_freq < 200000000) {
          F_Long = BIT_freq * 100ULL - F_Adjust(BIT_freq);
          freq_gen.set_freq(F_Long, SI5351_CLK2);
        }
        else Serial.println ("Frq out of range.");
      }
      break;

    case 'C': case 'c':
      Serial.println (clip_on);
      break;

    case 'D': case 'd':
      if (S_Input[1] == '?') Serial.println(F_Offset);
      else {
        F_Offset = LConv_Freq();
        writeLong(1, F_Offset);
 
        if (EXT_freq > 0) {
          //F_adjust(EXT_freq);
          F_Long = EXT_freq * 100ULL - F_Adjust(EXT_freq);
          freq_gen.set_freq(F_Long, SI5351_CLK1);
        }
        if (BIT_freq > 0) {
          //F_adjust(BIT_freq);
          F_Long = BIT_freq * 100ULL - F_Adjust(BIT_freq);
          freq_gen.set_freq(F_Long, SI5351_CLK2);
        }
        if (LO_freq > 0) {
          //F_adjust(LO_freq);
          F_Long = LO_freq * 100ULL - F_Adjust(LO_freq);
          freq_gen.set_freq(F_Long, SI5351_CLK0);
        }
 
      }
      break;

    case 'E': case 'e':
      if (S_Input[1] == '?') {
        if (EXT_freq < 10000000) Serial.print("0");
        Serial.println(EXT_freq);
      }
      else {
        EXT_freq = Conv_Freq();
        if (EXT_freq > 3999 && EXT_freq < 225000001) {
          F_Long = EXT_freq * 100ULL - F_Adjust(EXT_freq);
          freq_gen.set_freq(F_Long, SI5351_CLK1);
        }
        else Serial.println ("Frq out of range.");
      }
      break;

    case 'F': case 'f':
      if (S_Input[1] == '?') {
        if (LO_freq < 40000000) Serial.print("0");
        Serial.println(LO_freq >> 2);
      }
      else {
        LO_freq = Conv_Freq() * 4UL;
        if (LO_freq > 11999999 && LO_freq < 120000001) {
          Set_Band();
          F_Long = LO_freq * 100ULL - F_Adjust(LO_freq);
          if (LO_freq > 116000000){
            A_29 = 1;
            freq_gen.set_freq(F_Long, SI5351_CLK0);
          }
          else{
            if (A_29 == 1){
              A_29 = 0;
              freq_gen.set_pll(89999900000ULL, SI5351_PLLA);
            }
            freq_gen.set_freq_manual(F_Long, 89999900000ULL, SI5351_CLK0);
          }  
        }
        else Serial.println ("Frq out of range.");
      }
      break;

    case 'L': case 'l':
      Serial.print(analogRead(1));
      Serial.println();
      break;
      
    case 'O': case 'o':
      if (S_Input[1] == 'B' || S_Input[1] == 'b') clock_sel = SI5351_CLK2;
      if (S_Input[1] == 'E' || S_Input[1] == 'e') clock_sel = SI5351_CLK1;
      if (S_Input[1] == 'F' || S_Input[1] == 'f') clock_sel = SI5351_CLK0;
      c = S_Input[2] - 48;
      freq_gen.output_enable(clock_sel, c > 0 ? 1 : 0);
      switch (c) {
        default: drive = SI5351_DRIVE_2MA; break;
        case 2: drive = SI5351_DRIVE_4MA; break;
        case 3: drive = SI5351_DRIVE_6MA; break;
        case 4: drive = SI5351_DRIVE_8MA; break;
      }
      freq_gen.drive_strength(clock_sel, drive);
      break;

    case 'T': case 't':
      n = TEMP_C / 100;
      Serial.print (n);
      Serial.print (".");
      Serial.print (TEMP_C - (n * 100));
      Serial.println ("C");
      break;

    case 'W': case 'w':
      Serial.println (Version_Number);
      break;

    case 'X': case 'x':
      if (S_Input[1] == '0') TX_OFF();
      if (S_Input[1] == '1') TX_ON();
      break;
  }
}

void TX_ON(void){
  digitalWrite (EX_PTT, HIGH);
  digitalWrite (RX_HI, LOW);
  digitalWrite (TX_HI, HIGH);
  digitalWrite (DC_PTT, HIGH);
  digitalWrite (Y_LED, HIGH);
}

void TX_OFF(void){
  digitalWrite (TX_HI, LOW);
  digitalWrite (DC_PTT, LOW);
  digitalWrite (Y_LED, LOW);
  digitalWrite (RX_HI, HIGH);
  digitalWrite (EX_PTT, LOW);
}

unsigned long Conv_Freq(void) {
  unsigned long freq = 0, mult = 1;

  for (char i = 15; i > 0; i--) {
    if (S_Input[i] > 47 && S_Input[i] < 58) {
      freq += mult * (S_Input[i] - 48);
      mult *= 10;
    }
  }
  return freq;
}

long LConv_Freq(void) {
  long freq = 0, mult = 1;

  for (char i = 15; i > 0; i--) {
    if (S_Input[i] > 47 && S_Input[i] < 58) {
      freq += mult * (S_Input[i] - 48);
      mult *= 10;
    }
    if (S_Input[i] == '-') freq = -freq;
  }
  return freq;
}

void Set_Band (void) {
  byte B_freq = LO_freq / 4000000 ;
  HR50band = 0;
  switch (B_freq) {
    case 3: case 4: // 3 or 4 MHz = 80M
      band = 0;
      HR50band = 9;
      break;
    case 5:         // 5 MHz = 60M
      band = 1;
      HR50band = 8;
      break;
    case 6: case 7: // 6 or 7 MHz = 40M
      band = 2;
      HR50band = 7;
      break;
    case 10:        // 10 MHz = 30M
      band = 3;
      HR50band = 6;
      break;
    case 13: case 14: // 13 or 14 MHz = 20M
      band = 4;
      HR50band = 5;
      break;
    case 18:         // 18 MHz = 17M
      band = 5;
      HR50band = 4;
      break;
    case 20: case 21: // 20 or 21 MHz = 15M
      band = 6;
      HR50band = 3;
      break;
    case 24: case 25: // 24 or 25 MHz = 12M
      band = 7;
      HR50band = 2;
      break;
    case 27: case 28: case 29: // 27 - 29 MHz = 10M
      band = 8;
      HR50band = 1;
      break;
  }

  if (oldHRB != HR50band) {
    oldHRB = HR50band;
    if (HR50band == 0) {
      HR50.println ("HRBN10;");
    }
    else {
      HR50.write("HRBN0");
      HR50.write(HR50band + 48);
      HR50.println (";");
    }
  }

  clip = cl[band];
  digitalWrite (B80M, LOW);
  digitalWrite (B60_40M, LOW);
  digitalWrite (B30_20M, LOW);
  digitalWrite (B17_15M, LOW);
  digitalWrite (B12_10M, LOW);
  if (LO_freq < 17200000) {
    digitalWrite (B80M, HIGH);
    return;
  }
  if (LO_freq < 36000000) {
    digitalWrite (B60_40M, HIGH);
    return;
  }
  if (LO_freq < 60000000) {
    digitalWrite (B30_20M, HIGH);
    return;
  }
  if (LO_freq < 96000000) {
    digitalWrite (B17_15M, HIGH);
    return;
  }
  digitalWrite (B12_10M, HIGH);
}

long readLong(unsigned int addr) {
  union {
    byte b[4];
    long f;
  } data;
  for (int i = 0; i < 4; i++) {
    data.b[i] = EEPROM.read(addr + i);
  }
  return data.f;
}

void writeLong(unsigned int addr, long x) {
  union {
    byte b[4];
    long f;
  } data;
  data.f = x;
  for (int i = 0; i < 4; i++) {
    EEPROM.write(addr + i, data.b[i]);
  }
}

