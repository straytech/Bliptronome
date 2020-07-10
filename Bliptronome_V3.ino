/*
 *  Bliptronome_V3*
 * Translation of Arduinome firmware for Bliptronics 5000 
 * http://www.thinkgeek.com/electronics/musical-instruments/c4e1/
 * 
 * Wil Lindsay February 6th, 2010
 * http://www.straytechnologies.com for hardware mods, build information & port blog
 * 
 * Based on:
 * "ArduinomeFirmware3_2"  - With Revisions by 03/21/2009 Ben Southall
 * "ArduinomeFirmware" - Arduino Based Monome Clone by Owen Vallis & Jordan Hochenbaum 06/16/2008
 *  A translation of the 40h protocol designed by Brian Crabtree & Joe Lake, and coded by Joe Lake.
 *
 * changes in V2: Wil Lindsay February 16th, 2010
 * final reconfigured pinout to allow use 4 ADCs (4 used, 1 reserved, 1 Open)
 * final code additions of protocol commands: led_intensity, led_test & shutdown
 *
 * changes in V3: Sawyer Bernath June 21st 2012
 * converted for Arduino 1.0 syntax 
 *
 * --------------------------------------------------------------------------
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * --------------------------------------------------------------------------
 */

// Bliptronic specific pin assignments 
byte PORTD_Data_Direction = B11110010;
byte PORTB_Data_Direction = B00000001;
//byte PORTC_Data_Direction = B00000000;


#define IC_PORT (PORTD)  //pins to address HC164 & 74HC595D

byte U5_clock = 4; //pin D4
byte U5_clock_high = 1<< U5_clock;
byte U5_clock_low = ~U5_clock_high;

byte U7_clock = 5; //pin D5
byte U7_clock_high = 1<< U7_clock;
byte U7_clock_low = ~U7_clock_high;

byte U3_SH_CP = 6; //pin D6

byte U3_ST_CP = 7; //pin D7
byte U3_ST_CP_high = 1<< U3_ST_CP; 
byte U3_ST_CP_low = ~U3_ST_CP_high;

#define BLIP_SERIAL_PORT (PORTB)
byte blip_serial = 8; //pin D8
byte blip_serial_high = 1;//<< blip_serial;
byte blip_serial_low = ~blip_serial_high;

/* button read pins as follows
Col0 : D9
Col1 : D10
Col2 : D11
Col3 : D12
Col4 : D13
Col5 : D2    
Col6 : D3    
Col7 : D19(A5)
 */
 
#define BUTTON_READ_COL_0_4 (PINB)
// PORTB : col 0-4
#define BUTTON_READ_COL_5_6 (PIND)
// PORTD : col 5-6
#define BUTTON_READ_COL_7 (PINC)
// PORTC : col 7

// END pin and PORT assignments
// ----------------------------

// Global variables 

byte byte0, byte1;                      // storage for incoming serial data

byte WaitingForAddress = 1;             // 1 when we expect the next serial byte to be an address value, 0 otherwise

byte address =  0x00;                   // garbage byte to hold address of function
byte state = 0x00;                      // garbage byte to hold state value
byte x = 0x00;                          // garbage byte to hold x position
byte y = 0x00;                          // garbage byte to hold y position
byte z = 0x00;                          // garbage byte to iterate over buttons

// The following variables are used to store flags that indicate whether we have received a given message,
// the value of the data in that message.  e.g. IntensityChange = 0 == no intensity change message received,
// IntensityChange = 1 == an intensity change message has been received, and it's value will be in IntensityVal
// The code that eventually acts upon the incoming message will reset the 'Change' variable to 0 once the 
// message has been handled.

byte IntensityChange = 0;               
byte IntensityVal = 0; 
byte intensity = 15;

byte DisplayTestChange = 0;             
byte DisplayTestVal = 0;                

byte ShutdownModeChange = 0;            
byte ShutdownModeVal= 0;

// These variables are used to handle the ADC messages, and store which ports are currently enabled,
// and which are not.
byte ADCnum;
byte ADCstate;
byte ADCEnableState[4];

byte ledmem[8];                         // memory for LED states - 64 bits.

int b = 0;                              // temporary variable used in transmission of button presses/releases
int i = 0;                              // temporary variable for looping etc.
int id = 0;                             // temporary storage for incoming message ID
byte firstRun = 1;                      // 1 when we have yet to receive an LED command, 0 after that.
// used to ensure that our led state memory is properly initialized when we receive the first LED message

int kButtonUpDefaultDebounceCount   = 12;  // Used in button debouncing
int kButtonNewEvent   = 1;              // Used to store whether the state of a button has changed or not.
byte t = 0;                            // temporary variable used in button processing

/* BUTTON ARRAY VARIABLES  
 For 1/0 button states, use 8 bytes (64 bits)
 For button debounce counter, use 8x8 array
 */
byte button_current[8];
byte button_last[8];
byte button_state[8];
byte button_debounce_count[8][8];
byte button_event[8];
/* END OF BUTTON ARRAY VARIABLES */

void onLED(byte l_row,byte row_byte){      //address LEDs by row# (0-7) and Byte of on/off bits
 if (!ShutdownModeVal){
   for(int i=8;i>0;i--){  // cycle through all columns
    IC_PORT &= U5_clock_low; //U5_clock LOW
    if (i == l_row){     // if selected column
      BLIP_SERIAL_PORT |= blip_serial_high; //column HIGH
    }
    else{
      BLIP_SERIAL_PORT &= blip_serial_low; //else column LOW
    }
    IC_PORT |= U5_clock_high; //U5_clock HIGH
  }

  IC_PORT &= U3_ST_CP_low; //U3_ST_CP LOW
  if (DisplayTestVal){       //Override display values with all on
   shiftOut(blip_serial, U3_SH_CP, LSBFIRST,0);   //Allows all columns LOW (all LEDs powered)
  } else {
   shiftOut(blip_serial, U3_SH_CP, LSBFIRST,255-row_byte);  //LED Array bits LOW (selected LEDs on)
  } 
  IC_PORT |= U3_ST_CP_high; //U3_ST_CP HIGH  

  volatile int SlowDown = 0;           //direct PWM time increase based on IntensityVal
    while (SlowDown < intensity*16) { 
      SlowDown++; 
    }
  /*intensity needs corrected so that more
   intensity = IntensityVal * #ofbitsON in l_col
   currently less lights per row = brighter LEDS in that row 
   */

  IC_PORT &= U3_ST_CP_low; //U3_ST_CP LOW
  shiftOut(blip_serial, U3_SH_CP, MSBFIRST,255);   //blank the board
  IC_PORT |= U3_ST_CP_high; //U3_ST_CP HIGH 
 }
}

// buttonInit - helper function that zeros the button states
void buttonInit(void){
  byte i;
  for (i = 0; i < 8; i++) {
    button_current[i] = 0x00;
    button_last[i] = 0x00;
    button_state[i] = 0x00;
    button_event[i] = 0x00;
  }  
}


// buttonCheck - checks the state of a given button.
void buttonCheck(byte row, byte index)
{
  if (((button_current[row] ^ button_last[row]) & (1 << index)) &&   // if the current physical button state is different from the
  ((button_current[row] ^ button_state[row]) & (1 << index))) {  // last physical button state AND the current debounced state

    if (button_current[row] & (1 << index)) {                      // if the current physical button state is depressed
      button_event[row] = kButtonNewEvent << index;              // queue up a new button event immediately
      button_state[row] |= (1 << index);                         // and set the debounced state to down.
    }
    else{
      button_debounce_count[row][index] = kButtonUpDefaultDebounceCount;
    }  // otherwise the button was previously depressed and now
    // has been released so we set our debounce counter.
  }
  else if (((button_current[row] ^ button_last[row]) & (1 << index)) == 0 &&  // if the current physical button state is the same as
  (button_current[row] ^ button_state[row]) & (1 << index)) {        // the last physical button state but the current physical
    // button state is different from the current debounce 
    // state...

    if (button_debounce_count[row][index] > 0 && --button_debounce_count[row][index] == 0) {  // if the the debounce counter has
      // been decremented to 0 (meaning the
      // the button has been up for 
      // kButtonUpDefaultDebounceCount 
      // iterations///

      button_event[row] = kButtonNewEvent << index;    // queue up a button state change event

      if (button_current[row] & (1 << index)){          // and toggle the buttons debounce state.
        button_state[row] |= (1 << index);
      }
      else{
        button_state[row] &= ~(1 << index);
      }
    }
  }
}


void buttonpress () {
  for(i = 0; i < 8; i++){
    for (int b_col=0;b_col<8;b_col++){ 
      IC_PORT &= U7_clock_low;         
      if (b_col == i){
        BLIP_SERIAL_PORT |= blip_serial_high;
      }
      else{
        BLIP_SERIAL_PORT &= blip_serial_low;
      }
      IC_PORT |= U7_clock_high;
    }

//    // SlowDown No Longer Needed :kill time while output pins settle. 
//    volatile int SlowDown = 0; 
//    while (SlowDown < 15) { 
//      SlowDown++; 
//    } 

    button_last [i] = button_current [i]; 

    //****begin READ out ROW 
    for(id = 7; id > -1; id--) {
      switch(id){
        case 0:
          t = (BUTTON_READ_COL_0_4 & B00000010) >> 1;//read D9
          break;
        case 1:
          t = (BUTTON_READ_COL_0_4 & B00000100) >> 1;//read D10
          break;
        case 2:
          t = (BUTTON_READ_COL_0_4 & B00001000) >> 1;//read D11
          break;
        case 3:
          t = (BUTTON_READ_COL_0_4 & B00010000) >> 1;//read D12
          break;
        case 4:
          t = (BUTTON_READ_COL_0_4 & B00100000) >> 1;//read D13
          break;
        case 5:
          t = (BUTTON_READ_COL_5_6 & B00000100) >> 1;//read D2
          break;
        case 6:
          t = (BUTTON_READ_COL_5_6 & B00001000) >> 1;//read D3
          break;
        case 7:
          t = (BUTTON_READ_COL_7 & B00100000) >> 1;//read D19(A5)
          break;
      }
      if(t){
        button_current [i] |= (1 << id);
      }
      else{
        button_current [i] &= ~(1 << id);
      }

      buttonCheck(i, id);

      if (button_event[i] & (1 << id)) {
        button_event[i] &= ~(1 << id);
        if(button_state[i] & (1 << id)){
          b = 1;
        }
        else{
          b = 0;
        }
        Serial.write(byte((0 << 4) | (b & 15)));
        Serial.write(byte((id << 4) | (i & 15)));

      } 
    } //****** END READOUT ROW
  } 

}

ISR(TIMER2_OVF_vect) {
  // enable interrupts to keep the serial class responsive
  sei(); 

  do 
  {
    if (Serial.available())
    {
      if (WaitingForAddress == 1)
      {
        byte0 = Serial.read();

        address = byte0 >> 4;
        WaitingForAddress = 0;
      } // end if (WaitingForAddress == 1);

      if (Serial.available())
      {
        WaitingForAddress = 1;
        byte1 = Serial.read();

        switch(address)
        {
        case 2:
          state = byte0 & 15;
          x = byte1 >> 4;
          y = byte1 & 15;

          if (state == 0){
            ledmem[x] &= ~( 1 << y); 
          }
          else {
            ledmem[x] |=  ( 1 << y);
          }
          break;
        case 3:
          IntensityChange = 1;
          IntensityVal = byte1 & 15;          
          break;
        case 4:
          DisplayTestChange = 1;
          DisplayTestVal = byte1 & 15;
          break;
        case 5:
          state = byte1 & 0x0F;
          ADCEnableState[(byte1 >> 4)&0x03] = state;
          break;
        case 6:
          ShutdownModeChange = 1;
          ShutdownModeVal= byte1 & 15;
          break;
        case 7:
          if (firstRun == 1)
          {
            for (x = 0; x < 8; x++)
            {
              ledmem[x] = 0;
            }
            firstRun = 0;
          }
          x = ((byte0 & 15) & 0x7);
          y = byte1;
          for (z = 0; z < 8; z++)
          {
            if (y & (1 << z))
            {
              ledmem[z] |= 1 << x;
            }
            else
            {
              ledmem[z] &= ~(1 << x);
            }
          }
          break;
        case 8:
          if (firstRun == 1) {
            for (x = 0; x < 8; x++) {
              ledmem[x] = 0;
            }
            firstRun = 0;
          }
          x = ((byte0 & 15) & 0x7); // mask this value so we don't write to an invalid address.
          y = byte1;

          ledmem[x] = y;
          break;
        } // end switch(address)
      } // end if (Serial.available()
    } // end if (Serial.available();
  } // end do
  while (Serial.available() > 16);
}

void whoosh(void)  //LED init pattern
/* 
 Timer turned off in setup and on again just for this init function 
 */
{
  // reactivate overflow interrupt for
  // timer 1 - it's needed by delay(...)
  TIMSK0 |= (1 << TOIE0); 

  for (int j = 0; j < 9; j++)
  {
    for (int i = 0; i < 8; i++)
    {
      onLED(i+1,1<<(8-j));
    }
    delay(125);
  }
  // and switch the interrupt off.
  TIMSK0 &= ~(1 << TOIE0);
}


void setup () {

  DDRD = PORTD_Data_Direction;
  DDRB = PORTB_Data_Direction;

  Serial.begin(57600);

  buttonInit();  
  
  for (i=1; i<=8; i++){
    ledmem[i-1]=0;     
  }
  
  // Set up 8-bit counter 2, output compare switched off,
  // normal waveform generation (whatever that might mean)
  TCCR2A = 0;
  // set counter to be clocked at 16Mhz/8 = 2Mhz
  TCCR2B = 1<<CS21;

  // set the interrupt mask so that we get an interrupt
  // on timer 2 overflow, i.e. after 256 clock cycles.
  // The timer 2 interrupt routine will execute every
  // 128 uS.
  TIMSK2 = 1<<TOIE2;

  // NOTE: In my efforts to try to get this
  // code to work reliably at 115200 baud
  // I went about disabling unused timers that
  // are set up by the Arduino framework.
  // The framework sets up both timer 2 and 
  // timer 1.  We use timer 2 to drive the
  // serial reading interrupt, so we can only
  // disable timer1 and its interrupt.

  // DISABLE PWM COUNTER
  TIMSK0 &= ~(1 << TOIE0);

  TCCR1B &= ~(1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);  
  // END DISABLE PWM COUNTER

  // Also, disable the interrupts on timer 0
  // REALLY IMPORTANT NOTE IF YOU WANT TO USE
  // DELAY
  // remove this line, and also look at
  // 'whoosh', which enables and then disables
  // the intterupt on timer 0.  You'll want
  // to get rid of those lines too.
  TIMSK0 &= ~(1 << TOIE0);

  whoosh();
}  

void sendADC(int port, int value) {
  Serial.write(byte((1 << 4) | ((port << 2) & 0x0C) | ((value >> 8) & 0x03)));
  Serial.write(byte(value & 0xFF));
}

int current[4]; 
int previous[4]; 
int tolerance = 7;

void checkADCs() {

  for (int adcCounter = 0; adcCounter < 4; adcCounter++)
  {
    if (ADCEnableState[adcCounter] != 0)
    {
      current[adcCounter] = analogRead(adcCounter);
      if (abs(previous[adcCounter]-current[adcCounter]) > tolerance)
      {
        previous[adcCounter] = current[adcCounter];
        sendADC(adcCounter,current[adcCounter]);
      }
    }
  }
}


void loop () {

  // send the LED states to the LED matrix.
  for (int i = 0; i < 8; i++)
  {
    onLED(i+1,ledmem[i]);    
  }
  if (IntensityChange == 1)
  {
    IntensityChange = 0;
    intensity = IntensityVal & 15;
    //*  intensity is used directly via delay of IntensityVal in onLED*/
  }
  if (DisplayTestChange == 1)
  {
    DisplayTestChange = 0;
    //DisplayTestVal Directly used in onLED();
  }
  if (ShutdownModeChange == 1)
  {
    ShutdownModeChange = 0;
    //ShutdownModeVal Directly used in onLED();
  }
  
  // check for button presses
  buttonpress();

  // check the state of the ADCs
  checkADCs();  
}

