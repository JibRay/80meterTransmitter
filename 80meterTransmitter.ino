// 80meterTransmitter.ino
// This is a merge of iambicToneGenerator and localOscillator. It performs two
// transitter front-end functions: iambic key input and local oscillator
// frequency control.
//
/*
Iambic key:
  Input is a two-paddle key. output is an iambic tone to headphones.

  Processor is Arduino Nano, connected as follows (see defines below):

    TOME_PIN is tone output. It is connected to phone jack through a 750 ohm
    resistor and a 15 uF capacitor to ground.

    DIT_PIN is dit paddle input. Paddle grounds this pin when active.

    DAH_PIN is dah paddle input. Paddle grounds this pin when active.
  
  Rules:
  -Briefly pressing the dit paddle produces one dit tone.
  -Briefly pressing the dah paddle produces one dah tone.
  -Holding either paddle produces a continuous string of dits or dahs until the
   paddle is released.
  -Holding both paddles produces a string of dit-dah tones. The leading tone
   is dit if the dit paddle is pressed first or dah if the dah paddle is first.
   Releasing the dit paddle continues with a string of dahs, or dits if only
   the dah paddle is released.
  -All dits and dahs are followed by a period of silence equal to one dit
   time.

  Change TONE_FREQ to change the tone frequency.
  
  To change the timing, change the CODE_SPEED value below. All element timing
  is based on this value which is based on the 'PARIS' convention. The word
  PARIS contains 50 dit element equivalents. The silence between dits and
  dahs is one dit time. The dah time is three times as long as a dit.

  The program is two state machines. The main loop is a state machine that
  evaluates the paddle inputs. This passes the desired tone pattern to the
  tone state machine.

 Local Oscillator:
  Local oscillator commands set the frequency of the SI5351 device via
  the I2C interface.
 */

// Started by installing Adafruit Si5351 library. The
#include <Adafruit_SI5351.h>

//============================================================================
// Defines

#define BAUDRATE    19200
// These defines set the behavior of the iambic key:
#define CODE_SPEED   20                     // Words per minute.
#define TONE_FREQ    800                    // Hertz.

// Calculated dit and dah times. These are according to the 'PARIS' timing
// specs. The word PARIS contains 50 timing units.
#define WORD_TIME    60.0 / CODE_SPEED      // In seconds.
#define DIT_TIME     1000 * WORD_TIME / 50  // In msecs.
#define DAH_TIME     DIT_TIME * 3           // In msecs.

// Iambic key pin definitions:
#define LED_PIN      13  // Used only for debug.
#define TONE_PIN     12
#define DIT_PIN       2
#define DAH_PIN       3

#define IDLE_STATE    0
// Key states:
#define DIT_INPUT     1
#define DAH_INPUT     2
#define DIT_DAH_INPUT 3
#define DAH_DIT_INPUT 4
#define TONE_ON       1
#define TONE_PAUSE    2

// Tone types.
#define QUIET         0
#define DIT           1
#define DAH           2
#define DIT_DAH       3
#define DAH_DIT       4

// Command states.
#define CMD_ARGUMENT  1

// Argument states.
#define BUFFER_SIZE    80
#define S_COMMAND       1
#define C_VERSION       1
#define C_HELP          2
#define C_SET_FREQUENCY 3
#define C_PRINT_ID      4

//============================================================================
// Globals

const int version = 2;
Adafruit_SI5351 clockgen = Adafruit_SI5351();
char lineBuffer[BUFFER_SIZE];
int bufferIndex, mainState, commandState;
uint32_t LOfrequency;
int toneFrequency;
int keyState, toneState, lastTone, toneType;
unsigned long toneTimer, wordTime, ditTime, dahTime;

//============================================================================
// Global functions

void clearBuffer() {
  for (int i = 0; i < BUFFER_SIZE; ++i)
    lineBuffer[i] = '\0';
  bufferIndex = 0;
}

void printVersion() {
  Serial.print(F("80meterTransmitter version "));
  Serial.println(version);
}

// speed argument is words per minute.
void setWordSpeed(int speed) {
  unsigned long wordTime = (unsigned long)(60.0 / (float)speed);
  ditTime = (unsigned long)(1000.0 * (float)wordTime / 50.0);
  dahTime = (unsigned long)((float)ditTime * 3.0);
}

// Update the state of the iambic key state machine.
void updateKeyState() {
  bool ditKeyInput = digitalRead(DIT_PIN) == LOW;
  // debug
  digitalWrite(LED_PIN, ditKeyInput);
  // debug
  bool dahKeyInput = digitalRead(DAH_PIN) == LOW;
  
  switch(keyState) {
    case IDLE_STATE:
      if (ditKeyInput) {
        toneType = DIT;
        keyState = DIT_INPUT;
      } else if (dahKeyInput) {
        toneType = DAH;
        keyState = DAH_INPUT;
      } else {
        toneType = QUIET;
      }
      break;
    case DIT_INPUT:
      // If neither paddle is closed stop the tone. If the dah paddle is
      // now closed change the state to send repeating di-dah. Otherwise
      // continue to send dits.
      if (!ditKeyInput && !dahKeyInput) {
        toneType = QUIET;
        keyState = IDLE_STATE;
      } else if (ditKeyInput && dahKeyInput) {
        toneType = DIT_DAH;
        keyState = DIT_DAH_INPUT;
      }
      break;
    case DAH_INPUT:
      // If neither paddle is closed stop the tone. If the dit paddle is
      // now closed change state to send repeating dah-dit. Otherwise
      // continue to send dahs.
      if (!ditKeyInput && !dahKeyInput) {
        toneType = QUIET;
        keyState = IDLE_STATE;
      } else if (ditKeyInput && dahKeyInput) {
        toneType = DAH_DIT;
        keyState = DIT_DAH_INPUT;
      }
      break;
    case DIT_DAH_INPUT:
      // If either paddle is open switch back to sending dits or dahs by
      // by going to the idle state.
      if (!ditKeyInput || !dahKeyInput) {
        toneType = QUIET;
        keyState = IDLE_STATE;
      }
      break;
  }
}

// Update the state of the tone output. Depending on the toneState, send
// repeating dits, dahs or di-dahs. Each dit or dah is always followed by a
// quiet period (pause) equal to one dit time. Return true if at the end of
// a complete dit or dah tone including the following pause.
bool updateToneState(int toneType) {
  bool done = false;
  
  switch (toneState) {
    case IDLE_STATE:
      switch (toneType) {
        case DIT:
        case DIT_DAH:
          toneTimer = millis() + DIT_TIME;
          tone(TONE_PIN, TONE_FREQ);
          lastTone = DIT;
          toneState = TONE_ON;
          break;
        case DAH:
        case DAH_DIT:
          toneTimer = millis() + DAH_TIME;
          tone(TONE_PIN, TONE_FREQ);
          lastTone = DAH;
          toneState = TONE_ON;
          break;
      }
      break;
    case TONE_ON:
      if (millis() > toneTimer) {
        noTone(TONE_PIN);
        toneTimer = millis() + DIT_TIME;
        toneState = TONE_PAUSE;
      }
      break;
    case TONE_PAUSE:
      if (millis() > toneTimer) {
        switch (toneType) {
          case QUIET:
          case DIT:
          case DAH:
            toneState = IDLE_STATE;
            lastTone = QUIET;
            done = true;
            break;
          case DIT_DAH:
          case DAH_DIT:
            if (DIT == lastTone) {
              toneTimer = millis() + DAH_TIME;
              tone(TONE_PIN, TONE_FREQ);
              lastTone = DAH;
              toneState = TONE_ON;
            } else {
              toneTimer = millis() + DIT_TIME;
              tone(TONE_PIN, TONE_FREQ);
              lastTone = DIT;
              toneState = TONE_ON;
            }
            break;
        }
      }
      break;
  }
  return done;
}

// Update the local oscillator frequency state machine.
void updateOscillatorState() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    switch (mainState) {
      case IDLE_STATE:
        switch (c) {
          case 'v':
            commandState = C_VERSION;
            mainState = S_COMMAND;
            clearBuffer();
            break;
          case 'h':
            commandState = C_HELP;
            mainState = S_COMMAND;
            clearBuffer();
            break;
          case 'f':
            commandState = C_SET_FREQUENCY;
            mainState = S_COMMAND;
            clearBuffer();
            break;
          case '@':
            Serial.print(F("Local Oscillator"));
            commandState = C_PRINT_ID;
            mainState = S_COMMAND;
            clearBuffer();
            break;
        }
        break;
      case S_COMMAND:
        switch (commandState) {
          case C_VERSION:
            if (c == '\n' || c == '\r') {
              printVersion();
              commandState = IDLE_STATE;
              mainState = IDLE_STATE;
            }
            break;
          case C_HELP:
            if (c == '\n' || c == '\r') {
              printHelp();
              commandState = IDLE_STATE;
              mainState = IDLE_STATE;
            }
            break;
          case C_SET_FREQUENCY:
            if (c == '\n' || c == '\r') {
              setFrequency(atof(lineBuffer));
              commandState = IDLE_STATE;
              mainState = IDLE_STATE;
            } else {
              lineBuffer[bufferIndex++] = c;
            }
            break;
          case C_PRINT_ID:
            if (c == '\n' || c == '\r') {
              Serial.println();
              commandState = IDLE_STATE;
              mainState = IDLE_STATE;
            }
            break;
        }
        break;
    }
  }
}

// Set the local oscillator frequency.
void setFrequency(float f) {
//  float f = atof(lineBuffer);
  LOfrequency = (uint32_t)(f * 1e6);
  if ((LOfrequency >= 3500000) && (LOfrequency <= 4000000)) {
    Serial.print(F("Frequency set to "));
    Serial.println(LOfrequency);
    // Set PLL_B to 900 MHz.
    clockgen.setupPLL(SI5351_PLL_B, 36, 0, 1);
    float denominator = 25000000.0 * 36.0 / LOfrequency;
    uint32_t m1 = (uint32_t)denominator;
    uint32_t m2 = (uint32_t)(1000.0 * (denominator - (float)m1));
    clockgen.setupMultisynth(1, SI5351_PLL_B, m1, m2, 1000);
  } else {
    Serial.print(F("Error: invalid fequency = "));
    Serial.println(LOfrequency);
  }
}

void setup(void)
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(TONE_PIN, OUTPUT);
  pinMode(DIT_PIN, INPUT_PULLUP);
  pinMode(DAH_PIN, INPUT_PULLUP);

  Serial.begin(BAUDRATE);
  
  setWordSpeed(CODE_SPEED);
  toneFrequency = TONE_FREQ;
  keyState = toneState = IDLE_STATE;
  lastTone = QUIET;

  printVersion();
  clearBuffer();

  /* Initialise the clock */
  if (clockgen.begin() != ERROR_NONE)
  {
    Serial.println(F("Connection to Si5351 clock generator failed"));
    while (1);
  }
  mainState = IDLE_STATE;
  keyState = IDLE_STATE;
  commandState = IDLE_STATE;

  // INTEGER ONLY MODE --> most accurate output
  // Setup PLLA to integer only mode @ 900MHz (must be 600..900MHz)
  // Set Multisynth 0 to 112.5MHz using integer only mode (div by 4/6/8)
  // 25MHz * 36 = 900 MHz, then 900 MHz / 8 = 112.5 MHz
  Serial.println("Set PLLA to 900MHz");
  clockgen.setupPLLInt(SI5351_PLL_A, 36);
  Serial.println("Set Output #0 to 112.5MHz");
  clockgen.setupMultisynthInt(0, SI5351_PLL_A, SI5351_MULTISYNTH_DIV_8);

  // My code:
  // Serial.println("Set PLLA to 600MHz");
  // clockgen.setupPLLInt(SI5351_PLL_A, 24);
  // Serial.println("Set Output #0 to 75MHz");
  // clockgen.setupMultisynthInt(0, SI5351_PLL_A, SI5351_MULTISYNTH_DIV_8);

  // FRACTIONAL MODE --> More flexible but introduce clock jitter
  // Setup PLLB to fractional mode @616.66667MHz (XTAL * (24 + 2/3))
  // Setup Multisynth 1 to 13.55311MHz (PLLB/45.5)
  // clockgen.setupPLL(SI5351_PLL_B, 24, 2, 3);
  // Serial.println("Set Output #1 to 13.553115MHz");
  // clockgen.setupMultisynth(1, SI5351_PLL_B, 45, 1, 2);

  // My code:
  // clockgen.setupPLL(SI5351_PLL_B, p1, p2, p3);
  // clockgen.setupMultisynth(1, SI5351_PLL_B, m1, m2, m3);
  // f = (25e6 * (p1 + p2/p3)) / (m1 + (m2/m3))
  // clockgen.setupPLL(SI5351_PLL_B, 36, 0, 1);
  // Serial.println("Set Output #1 to 350.0 MHz");
  // clockgen.setupMultisynth(1, SI5351_PLL_B, 257, 1, 7);


  // Multisynth 2 is not yet used and won't be enabled, but can be
  // Use PLLB @ 616.66667MHz, then divide by 900 -> 685.185 KHz
  // then divide by 64 for 10.706 KHz
  // configured using either PLL in either integer or fractional mode

  Serial.println("Set Output #2 to 10.706 KHz");
  clockgen.setupMultisynth(2, SI5351_PLL_B, 900, 0, 1);
  clockgen.setupRdiv(2, SI5351_R_DIV_64);

  setFrequency(3.500000);
  // Enable the clocks
  clockgen.enableOutputs(true);
}

void printHelp() {
  Serial.println();
  Serial.println(F("Commands are 'Norm' or 'Imed'. Norm commands require an"));
  Serial.println(F("Enter character following input. Imed commands are"));
  Serial.println(F("executed when the command letter is received."));
  Serial.println(F("Command  Argument    Type  Description"));
  Serial.println(F("--------+-----------+-----+----------------------------"));
  Serial.println(F("  v                  Norm  Print version."));
  Serial.println(F("  h                  Norm  Print this help."));
  Serial.println(F("  f      frequency   Norm  Set fequency in megahertz."));
  Serial.println(F("                           Must be between 3.5 and 4.0."));
  Serial.println(F("  @                  Imed  Print device ID"));
}

//============================================================================
// Main loop

void loop(void)
{
  updateKeyState();
  updateToneState(toneType);
  updateOscillatorState();
}
