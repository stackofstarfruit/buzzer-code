//BUZZER PROGRAM CODE//
// AUDIO LIBRARY
#include <SD.h>
#include <SPI.h>
#include <Audio.h>
#include <malloc.h>

// DEBUGGING MATERIALS
#define SOFTWAREDEBUG
#ifdef SOFTWAREDEBUG
#define PRINTBUFSIZE 2000
volatile bool gotStuckInRung = false; // triggers if sort after clear is triggered incorrectly
volatile bool fired[9] = {false, false, false, false, false, false, false, false, false};
volatile unsigned long firstHalfTimes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile unsigned long secondHalfTimes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile char* contents[300];
volatile int clears = 0;
char printbuf[PRINTBUFSIZE];
char strbuf[200];

void BufferAdd(char *add) {
  if ((strlen(printbuf) + strlen(add) + 1) < PRINTBUFSIZE) strcat(printbuf, add);
}

void BufferPrint(void) {
  Serial.print(printbuf);
  Serial.flush();
  printbuf[0] = '\0'; // clear the buffer
}
#endif

/*
//#define HARDWAREDEBUG
#ifdef HARDWAREDEBUG
const int R1debugpin = 23;
const int G1debugpin = 27;
const int Cdebugpin = 31;
const int Sounddebugpin = 35;
#endif

inline void digitalWriteD(int pin, boolean val){
  if(val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

inline int digitalReadD(int pin){
  return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
}

inline void fastToggleD(int pin){
  digitalWriteD(pin, !(digitalReadD(pin)));
}

extern char _end;
extern "C" char *sbrk(int i);
char *ramstart = (char *)0x20070000;
char *ramend = (char *)0x20088000;
*/

// AUDIO FILES
File bzA;
File bzB;
bool soundDone = false;

// PIN VALUES
const int numInputs = 9;
const int numLights = 10;
// pins for actual setup

const int sdPin = 10;
const int mutePin = 32;
const int buttons[9] = {25, 28, 26, 29, 27, 31, 33, 34, 35};
const int lights[10] = {4, 8, 6, 7, 5, 13, 11, 12, 9, 3}; // (8, 13)
volatile int foundIndex = 0;

// pins for test setup
/*
const int sdPin = 4;
const int mutePin = 52;
const int buttons[9] = {44, 48, 30, 31, 32, 49, 33, 34, 35};
const int lights[10] = {3, 6, 7, 8, 9, 5, 11, 12, 13, 5};
*/

// TIMING VARIABLES
const unsigned long MAX_VAL = 0 - 1;
unsigned long SETUP_TIME;
volatile unsigned long lastSuperClear = 0;
volatile unsigned long bigLastPresses[9];
volatile unsigned long timeOfBuzz = 0;
// lastPresses is declared inside of interrupt_handler_clear ISR

bool dimmed = false;

// INTERRUPT VARIABLES
bool clearState = false;
const unsigned long debounceDelay = 50000;
const unsigned long interruptDelay = 5000;
volatile bool isAttached[9] = {true, true, true, true, true, true, true, true, true};
volatile bool isRunning[9] = {false, false, false, false, false, false, false, false, false};
volatile unsigned long lastAttachTimes[9];
volatile bool hasBuzzed[9] = {false, false, false, false, false, false, false, false, false}; // prevents people from buzzing again before super clear

// LOCKOUT SYSTEM
volatile bool redOn = true;
volatile bool greenOn = true;
volatile bool locked = true;
volatile bool dimLight = false;
volatile bool isDim = false;
volatile bool noDim = false;
bool lightsOff = false;

void setup() {

  printbuf[0] = '\0';
  
  // Initialize SD Card
  pinMode(30, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(39, OUTPUT);
  
  Serial.begin(9600);
  Serial.print("Initializing SD card...");
  if (!SD.begin(sdPin)) { // change to 10 for actual board
    Serial.println(" failed!");
    while(true);
  }
  Serial.println(" done.");

  // Initialize file objects
  bzA = SD.open("bzA.wav");
  bzB = SD.open("bzB.wav");
  if (!bzA) {
    Serial.println("bzA.wav could not be opened");
    while (true);
  } else if(!bzB) {
    Serial.println("bzB.wav could not be opened");
    while (true);
  }
  
  // writes initial pin states
  pinMode(buttons[0], INPUT_PULLUP);
  for(int i = 1; i < numInputs; i++) {
    pinMode(buttons[i], INPUT);
  }
  
  for(int i = 0; i < numLights; i++) {
    pinMode(lights[i], OUTPUT);
    analogWrite(lights[i], 30);
  }

  #ifdef HARDWAREDEBUG
  pinMode(R1debugpin, OUTPUT);
  digitalWriteD(R1debugpin, 0);
  pinMode(G1debugpin, OUTPUT);
  digitalWriteD(G1debugpin, 0);
  pinMode(Cdebugpin, OUTPUT);
  digitalWriteD(G1debugpin, 0);
  pinMode(Sounddebugpin, OUTPUT);
  digitalWriteD(Sounddebugpin, 0);
  #endif

  SETUP_TIME = micros();
  for(int k = 0; k < numInputs; k++) {
    lastAttachTimes[k] = SETUP_TIME;
  }
  for(int k = 0; k < numInputs; k++) {
    hasBuzzed[k] = false;
  }

  // attaches interrupts to all buttons
  attachInterrupt(digitalPinToInterrupt(buttons[0]), interrupt_handler_clear, CHANGE);
  attachInterrupt(digitalPinToInterrupt(buttons[1]), interrupt_handler_R1, RISING);
  attachInterrupt(digitalPinToInterrupt(buttons[2]), interrupt_handler_R2, RISING);
  attachInterrupt(digitalPinToInterrupt(buttons[3]), interrupt_handler_R3, RISING);
  attachInterrupt(digitalPinToInterrupt(buttons[4]), interrupt_handler_R4, RISING);
  attachInterrupt(digitalPinToInterrupt(buttons[5]), interrupt_handler_G1, RISING);
  attachInterrupt(digitalPinToInterrupt(buttons[6]), interrupt_handler_G2, RISING);
  attachInterrupt(digitalPinToInterrupt(buttons[7]), interrupt_handler_G3, RISING);
  attachInterrupt(digitalPinToInterrupt(buttons[8]), interrupt_handler_G4, RISING);

  bigLastPresses[0] = SETUP_TIME;
  for(int k = 1; k < numInputs; k++) {
    bigLastPresses[k] = MAX_VAL;
  }
  
  delay(1000);

  pinMode(mutePin, OUTPUT);
  digitalWrite(mutePin, HIGH);

  Audio.begin(44100, 100);
}


void reattachInterrupt(uint32_t pin)
{
  // Retrieve pin information
  Pio *pio = g_APinDescription[pin].pPort;
  uint32_t mask = g_APinDescription[pin].ulPin;
  
  // Enable interrupt
  pio->PIO_IER = mask;
}

// interrupt handler for clear button
void interrupt_handler_clear() {

  noInterrupts();
  
  if (!isRunning[0] && isAttached[0]) {
    isRunning[0] = true;
    unsigned long interruptClear;
    interruptClear = micros();
    if(interruptClear - lastAttachTimes[0] > interruptDelay) {
      detachInterrupt(digitalPinToInterrupt(buttons[0]));
      bigLastPresses[0] = interruptClear;
      isAttached[0] = false;
      
      #ifdef SOFTWAREDEBUG
      fired[0] = true;
      sprintf(strbuf,"%u: DETACH CLEAR\n",interruptClear);
      BufferAdd(strbuf);
      #endif
      
      if(clearState) {
        // DO NOTHING; BUZZ IS A RELEASE, NOT A PRESS
        clearState = false;
      } else {
        #ifdef HARDWAREDEBUG
        fastToggleD(Cdebugpin);
        #endif
        // resets lights and locked booleans
        dimLight = false;
        isDim = false;
        redOn = false;
        greenOn = false;
        locked = false;
        noDim = true;
        for(int i = 0; i < numLights; i++) {
          analogWrite(lights[i], 0);
        }
    
        // checks if any button is in pressed state
        bool isClear = true;
        bool states[9];
  
        for(int i = 1; i < numInputs; i++) { // skips clear button
          if((digitalRead(buttons[i]) == HIGH) && (!hasBuzzed[i])) {
            states[i] = true;
            isClear = false;
          } else {
            states[i] = false;
          }
        }

        // clears array of interrupt times if no button is pressed, finds next buzz if not
        if(isClear) {
          lastSuperClear = micros();
          for(int l = 1; l < numInputs; l++) { // skip clear
            bigLastPresses[l] = MAX_VAL;
            hasBuzzed[l] = false;
          }
        } else {       
          
          // MERGE SORT
          // first iteration of merge sort
          unsigned long lastPresses[8];
          int lastIndices[8];
          noInterrupts();
          for(int i = 0; i < 8; i++) {
            lastPresses[i] = bigLastPresses[i+1] - lastSuperClear;
            lastIndices[i] = i + 1;
            
            #ifdef SOFTWAREDEBUG
            //sprintf(strbuf,"ORIGINAL: lastPresses[%i]=%u\n",i,lastPresses[i]);
            //BufferAdd(strbuf);
            #endif
          }
          interrupts();
  
          for(int i = 0; i < 8; i += 2) {
            if(lastPresses[i] > lastPresses[i+1]) {
              int tempVal = lastIndices[i];
              lastIndices[i] = lastIndices[i+1];
              lastIndices[i+1] = tempVal;
            }
          }

          // second iteration of merge sort
          int newArray[8] = {0, 0, 0, 0, 0, 0, 0, 0};
          int ind = 0;
          for(int p = 0; p < 5; p += 4) {
            int pointerOne = p;
            int pointerTwo = p + 2;
            while((pointerOne < (p + 2)) && (pointerTwo < (p + 4))) {
              if(lastPresses[lastIndices[pointerOne] - 1] > lastPresses[lastIndices[pointerTwo] - 1]) {
                newArray[ind] = lastIndices[pointerTwo];
                pointerTwo++;
              } else {
                newArray[ind] = lastIndices[pointerOne];
                pointerOne++;
              }
              ind++;
            }
            
            if(pointerOne == p + 2) {
              while(pointerTwo < (p + 4)) {
                newArray[ind] = lastIndices[pointerTwo];
                pointerTwo++;
                ind++;
              }
            } else {
              while(pointerOne < (p + 2)) {
                newArray[ind] = lastIndices[pointerOne];
                pointerOne++;
                ind++;
              }
            }
          }
        
          for(int k = 0; k < 8; k++) {
            lastIndices[k] = newArray[k];

            #ifdef SOFTWAREDEBUG
            //sprintf(strbuf,"lastIndices[%i]=%i TIME: %u\n",k,lastIndices[k],lastPresses[lastIndices[k] - 1]);
            //BufferAdd(strbuf);
            #endif
          }

  
          // third iteration of merge sort
          ind = 0;
          int newPointerOne = 0;
          int newPointerTwo = 4;
          while(newPointerOne < 4 && newPointerTwo < 8) {
            
            
            /*firstHalfTimes[ind] = lastPresses[lastIndices[newPointerOne] - 1];
            secondHalfTimes[ind] = lastPresses[lastIndices[newPointerTwo] - 1];

            sprintf(strbuf,"FIRST arr[%i]=%u\n",ind,firstHalfTimes[ind]);
            BufferAdd(strbuf);
            sprintf(strbuf,"SECOND arr[%i]=%u\n",ind,secondHalfTimes[ind]);
            BufferAdd(strbuf);*/
            
            if(lastPresses[lastIndices[newPointerOne] - 1] > lastPresses[lastIndices[newPointerTwo] - 1]) {
              newArray[ind] = lastIndices[newPointerTwo];
              newPointerTwo++;
            } else {
              newArray[ind] = lastIndices[newPointerOne];
              newPointerOne++;
            }
            ind++;
          }
          
          if(newPointerOne == 4) {
            while(newPointerTwo < 8) {
              newArray[ind] = lastIndices[newPointerTwo];
              newPointerTwo++;
              ind++;
            }
          } else {
            while(newPointerOne < 4) {
              newArray[ind] = lastIndices[newPointerOne];
              newPointerOne++;
              ind++;
            }
          }
          for(int k = 0; k < 8; k++) {
            lastIndices[k] = newArray[k];
          }
          // END MERGE SORT
          
          for(int j = 0; j < 8; j++) {
            int index = lastIndices[j];
            if(states[index] && (!hasBuzzed[index])) {
              foundIndex = index;
              goto exitHere;
            }
          }
          
          gotStuckInRung = true; // marks case where button not found
          Serial.println("YOU GOT STUCK");
          
          exitHere:
          int currLight = lights[foundIndex];
          analogWrite(currLight, 200);
          if(foundIndex < 5) {
            redOn = true;
            analogWrite(lights[0], 60);
          } else {
            greenOn = true;
            analogWrite(lights[9], 60);
          }
          hasBuzzed[foundIndex] = true;
        } // else
        clears++;
      }
    }
    isRunning[0] = false;
  }
  interrupts();
}

// Interrupt handlers for red buttons
void interrupt_handler_R1() {
  noInterrupts();
  unsigned long interruptR1;
  if(!isRunning[1] && isAttached[1]) {
    isRunning[1] = true;
    interruptR1 = micros();
    detachInterrupt(digitalPinToInterrupt(buttons[1]));
    isAttached[1] = false;

    #ifdef SOFTWAREDEBUG
    sprintf(strbuf,"%u: DETACH R1\n",interruptR1);
    BufferAdd(strbuf);
    #endif
    
    if(interruptR1 - lastAttachTimes[1] > interruptDelay) {
      #ifdef HARDWAREDEBUG
      fastToggleD(R1debugpin);
      #endif
      if(!redOn && !greenOn) {
        redOn = true;
        analogWrite(lights[0], 60);
        analogWrite(lights[1], 200);
        hasBuzzed[1] = true;
        foundIndex = 1;
      }
      bigLastPresses[1] = interruptR1;
      
      #ifdef SOFTWAREDEBUG
      fired[1] = true;
      #endif
    }
    isRunning[1] = false;
  }
  interrupts();
}

void interrupt_handler_R2() {
  noInterrupts();
  unsigned long interruptR2;
  if (!isRunning[2] && isAttached[2]) {
    isRunning[2] = true;
    interruptR2 = micros();
    detachInterrupt(digitalPinToInterrupt(buttons[2]));
    isAttached[2] = false;

    #ifdef SOFTWAREDEBUG
    sprintf(strbuf,"%u: DETACH R2\n",interruptR2);
    BufferAdd(strbuf);
    #endif
    
    if(interruptR2 - lastAttachTimes[2] > interruptDelay) {
      #ifdef HARDWAREDEBUG
      fastToggleD(R2debugpin);
      #endif
      if(!redOn && !greenOn) {
        redOn = true;
        analogWrite(lights[0], 60);
        analogWrite(lights[2], 200);
        hasBuzzed[2] = true;
        foundIndex = 2;
      }
      bigLastPresses[2] = interruptR2;

      #ifdef SOFTWAREDEBUG
      fired[2] = true;
      #endif
    }
    isRunning[2] = false;
  }
  interrupts();
}

void interrupt_handler_R3() {
  noInterrupts();
  unsigned long interruptR3;
  if (!isRunning[3] && isAttached[3]) {
    isRunning[3] = true;
    interruptR3 = micros();
    detachInterrupt(digitalPinToInterrupt(buttons[3]));
    isAttached[3] = false;

    #ifdef SOFTWAREDEBUG
    sprintf(strbuf,"%u: DETACH R3\n",interruptR3);
    BufferAdd(strbuf);
    #endif
    
    if(interruptR3 - lastAttachTimes[3] > interruptDelay) {
      #ifdef HARDWAREDEBUG
      fastToggleD(R3debugpin);
      #endif
      if(!redOn && !greenOn) {
        redOn = true;
        analogWrite(lights[0], 60);
        analogWrite(lights[3], 200);
        hasBuzzed[3] = true;
        foundIndex = 3;
      }
      bigLastPresses[3] = interruptR3;

      #ifdef SOFTWAREDEBUG
      fired[3] = true;
      #endif
    }
    isRunning[3] = false;
  }
  interrupts();
}

void interrupt_handler_R4() {
  noInterrupts();
  unsigned long interruptR4;
  if (!isRunning[4] && isAttached[4]) {
    isRunning[4] = true;
    interruptR4 = micros();
    detachInterrupt(digitalPinToInterrupt(buttons[4]));
    isAttached[4] = false;

    #ifdef SOFTWAREDEBUG
    sprintf(strbuf,"%u: DETACH R4\n",interruptR4);
    BufferAdd(strbuf);
    #endif
    
    if(interruptR4 - lastAttachTimes[4] > interruptDelay) {
      #ifdef HARDWAREDEBUG
      fastToggleD(R4debugpin);
      #endif
      if(!redOn && !greenOn) {
        redOn = true;
        analogWrite(lights[0], 60);
        analogWrite(lights[4], 200);
        hasBuzzed[4] = true;
        foundIndex = 4;
      }
      bigLastPresses[4] = interruptR4;

      #ifdef SOFTWAREDEBUG
      fired[4] = true;
      #endif
    }
    isRunning[4] = false;
  }
  interrupts();
}

// Interrupt handlers for green buttons
void interrupt_handler_G1() {
  noInterrupts();
  unsigned long interruptG1;
  if (!isRunning[5] && isAttached[5]) {
    isRunning[5] = true;
    interruptG1 = micros();
    detachInterrupt(digitalPinToInterrupt(buttons[5]));
    isAttached[5] = false;

    #ifdef SOFTWAREDEBUG
    sprintf(strbuf,"%u: DETACH G1\n",interruptG1);
    BufferAdd(strbuf);
    #endif
    
    if(interruptG1 - lastAttachTimes[5] > interruptDelay) {
      #ifdef HARDWAREDEBUG
      fastToggleD(G1debugpin);
      #endif
      if(!redOn && !greenOn) {
        greenOn = true;
        analogWrite(lights[9], 60);
        analogWrite(lights[5], 200);
        hasBuzzed[5] = true;
        foundIndex = 5;
      }
      bigLastPresses[5] = interruptG1;

      #ifdef SOFTWAREDEBUG
      fired[5] = true;
      #endif
    }
    isRunning[5] = false;
  }
  interrupts();
}

void interrupt_handler_G2() {
  noInterrupts();
  unsigned long interruptG2;
  if (!isRunning[6] && isAttached[6]) {
    isRunning[6] = true;
    interruptG2 = micros();
    detachInterrupt(digitalPinToInterrupt(buttons[6]));
    isAttached[6] = false;

    #ifdef SOFTWAREDEBUG
    sprintf(strbuf,"%u: DETACH G2\n",interruptG2);
    BufferAdd(strbuf);
    #endif
    
    if(interruptG2 - lastAttachTimes[6] > interruptDelay) {
      #ifdef HARDWAREDEBUG
      fastToggleD(G2debugpin);
      #endif
      if(!redOn && !greenOn) {
        greenOn = true;
        analogWrite(lights[9], 60);
        analogWrite(lights[6], 200);
        hasBuzzed[6] = true;
        foundIndex = 6;
      }
      bigLastPresses[6] = interruptG2;

      #ifdef SOFTWAREDEBUG
      fired[6] = true;
      #endif
    }
    isRunning[6] = false;
  }
  interrupts();
}

void interrupt_handler_G3() {
  noInterrupts();
  unsigned long interruptG3;
  if (!isRunning[7] && isAttached[7]) {
    isRunning[7] = true;
    interruptG3 = micros();
    detachInterrupt(digitalPinToInterrupt(buttons[7]));
    isAttached[7] = false;

    #ifdef SOFTWAREDEBUG
    sprintf(strbuf,"%u: DETACH G3\n",interruptG3);
    BufferAdd(strbuf);
    #endif
    
    if(interruptG3 - lastAttachTimes[7] > interruptDelay) {
      #ifdef HARDWAREDEBUG
      fastToggleD(G3debugpin);
      #endif
      if(!redOn && !greenOn) {
        greenOn = true;
        analogWrite(lights[9], 60);
        analogWrite(lights[7], 200);
        hasBuzzed[7] = true;
        foundIndex = 7;
      }
      bigLastPresses[7] = interruptG3;

      #ifdef SOFTWAREDEBUG
      fired[7] = true;
      #endif
    }
    isRunning[7] = false;
  }
  interrupts();
}

void interrupt_handler_G4() {
  noInterrupts();
  unsigned long interruptG4;
  if (!isRunning[8] && isAttached[8]) {
    isRunning[8] = true;
    interruptG4 = micros();
    detachInterrupt(digitalPinToInterrupt(buttons[8]));
    isAttached[8] = false;

    #ifdef SOFTWAREDEBUG
    sprintf(strbuf,"%u: DETACH G4\n",interruptG4);
    BufferAdd(strbuf);
    #endif

    if(interruptG4 - lastAttachTimes[8] > interruptDelay) {
      #ifdef HARDWAREDEBUG
      fastToggleD(G4debugpin);
      #endif
      if(!redOn && !greenOn) {
        greenOn = true;
        analogWrite(lights[9], 60);
        analogWrite(lights[8], 200);
        hasBuzzed[8] = true;
        foundIndex = 8;
      }
      bigLastPresses[8] = interruptG4;

      #ifdef SOFTWAREDEBUG
      fired[8] = true;
      #endif
    }
    isRunning[8] = false;
  }
  interrupts();
}

void loop() {
  //#define REATTACHDEBUG
  
  #ifdef SOFTWAREDEBUG
  /*for(int i = 0; i < numInputs; i++) {
    if(fired[i]) {
      Serial.print(bigLastPresses[i]);
      Serial.print(":INDEX ");
      Serial.print(i);
      Serial.println(" DETACHED");
      
      fired[i] = false;
    }
  }*/

  /*if(clears == 2) {
    BufferPrint();
  }*/

  /*Serial.println("LAST CLEAR PRESS");
  Serial.println(bigLastPresses[0]);
  Serial.println("hasBuzzed");
  Serial.println(hasBuzzed[1]);
  Serial.println("CLEAR STATE");
  Serial.println(clearState);*/

  char str1[200];
  /*for(int i = 0; i < numInputs; i++) {
    sprintf(str1,"bigLastPresses[%i]=%u",i,bigLastPresses[i]);
    Serial.println(str1); 
  }*/
  #endif

  
  // REATTACH INTERRUPTS
  unsigned long currMicros = micros();
  if((!isAttached[0]) && ((currMicros - bigLastPresses[0]) > debounceDelay) && (!isRunning[0]) && soundDone) {
     lastAttachTimes[0] = currMicros;
     isAttached[0] = true;
     if(digitalRead(buttons[0]) == LOW) {
        clearState = true;
     } else {
        clearState = false;
     }
     
     #ifdef SOFTWAREDEBUG
     //Serial.println("SOUND DONE vv");
     //Serial.println(soundDone);
     #ifdef REATTACHDEBUG
     sprintf(str1,"%u:ClearReattached\n",currMicros);
     BufferAdd(str1);
     #endif
     #endif
     
     reattachInterrupt(digitalPinToInterrupt(buttons[0]));
  }
  if((!isAttached[1]) && (digitalRead(buttons[1]) == LOW) && ((currMicros - bigLastPresses[1]) > debounceDelay) && (!isRunning[1])) {
      lastAttachTimes[1] = currMicros;
      isAttached[1] = true;
      
      #ifdef REATTACHDEBUG
      sprintf(str1,"%u:R1Reattached\n",currMicros);
      BufferAdd(str1);
      #endif
      
      reattachInterrupt(digitalPinToInterrupt(buttons[1]));
  }
  if((!isAttached[2]) && (digitalRead(buttons[2]) == LOW) && ((currMicros - bigLastPresses[2]) > debounceDelay) && (!isRunning[2])) {
      lastAttachTimes[2] = currMicros;
      isAttached[2] = true;
      
      #ifdef REATTACHDEBUG
      sprintf(str1,"%u:R2Reattached\n",currMicros);
      BufferAdd(str1);
      #endif
      
      reattachInterrupt(digitalPinToInterrupt(buttons[2]));
  }
  if((!isAttached[3]) && (digitalRead(buttons[3]) == LOW) && ((currMicros - bigLastPresses[3]) > debounceDelay) && (!isRunning[3])) {
      lastAttachTimes[3] = currMicros;
      isAttached[3] = true;

      #ifdef REATTACHDEBUG
      sprintf(str1,"%u:R3Reattached\n",currMicros);
      BufferAdd(str1);
      #endif
      
      reattachInterrupt(digitalPinToInterrupt(buttons[3]));
  }
  if((!isAttached[4]) && (digitalRead(buttons[4]) == LOW) && ((currMicros - bigLastPresses[4]) > debounceDelay) && (!isRunning[4])) {
      lastAttachTimes[4] = currMicros;
      isAttached[4] = true;
      
      #ifdef REATTACHDEBUG
      sprintf(str1,"%u:R4Reattached\n",currMicros);
      BufferAdd(str1);
      #endif
      
      reattachInterrupt(digitalPinToInterrupt(buttons[4]));
  }
  if((!isAttached[5]) && (digitalRead(buttons[5]) == LOW) && ((currMicros - bigLastPresses[5]) > debounceDelay) && (!isRunning[5])) {
      lastAttachTimes[5] = currMicros;
      isAttached[5] = true;
      
      #ifdef REATTACHDEBUG
      sprintf(str1,"%u:G1Reattached\n",currMicros);
      BufferAdd(str1);
      #endif
      
      reattachInterrupt(digitalPinToInterrupt(buttons[5]));
  }
  if((!isAttached[6]) && (digitalRead(buttons[6]) == LOW) && ((currMicros - bigLastPresses[6]) > debounceDelay) && (!isRunning[6])) {
      lastAttachTimes[6] = currMicros;
      isAttached[6] = true;
      
      #ifdef REATTACHDEBUG
      sprintf(str1,"%u:G2Reattached\n",currMicros);
      BufferAdd(str1);
      #endif
      
      reattachInterrupt(digitalPinToInterrupt(buttons[6]));
  }
  if((!isAttached[7]) && (digitalRead(buttons[7]) == LOW) && ((currMicros - bigLastPresses[7]) > debounceDelay) && (!isRunning[7])) {
      lastAttachTimes[7] = currMicros;
      isAttached[7] = true;
      
      #ifdef REATTACHDEBUG
      sprintf(str1,"%u:G3Reattached\n",currMicros);
      BufferAdd(str1);
      #endif
      
      reattachInterrupt(digitalPinToInterrupt(buttons[7]));
  }
  if((!isAttached[8]) && (digitalRead(buttons[8]) == LOW) && ((currMicros - bigLastPresses[8]) > debounceDelay) && (!isRunning[8])) {
      lastAttachTimes[8] = currMicros;
      isAttached[8] = true;
      
      #ifdef REATTACHDEBUG
      sprintf(str1,"%u:G4Reattached\n",currMicros);
      BufferAdd(str1);
      #endif
      
      reattachInterrupt(digitalPinToInterrupt(buttons[8]));
  }

  // Buzzer sound event
  if(!locked && (redOn || greenOn)) {
    locked = true;
    timeOfBuzz = micros();
    lightsOff = false;
    
    const int S = 1024; // Number of samples to read in block
    short buffer[S];

    soundDone = false;

    Audio.end();
    Audio.begin(44100, 100);
    
    digitalWrite(mutePin, LOW);
    delay(50);

    if(redOn) {
      bzA.seek(0);
     
      while (bzA.available()) {
        digitalWrite(30, HIGH);
        #ifdef HARDWAREDEBUG
        fastToggleD(Sounddebugpin);
        #endif

        // read from the file into buffer
        bzA.read(buffer, sizeof(buffer));
        
        // Prepare samples 
        int volume = 1024;
        Audio.prepare(buffer, S, volume);
        
        // Play sound
        Audio.write(buffer, S);
      }       
    } else if(greenOn) {
      bzB.seek(0);
      while (bzB.available()) {
        #ifdef HARDWAREDEBUG
        fastToggleD(Sounddebugpin);
        #endif
        // read from the file into buffer
        bzB.read(buffer, sizeof(buffer));
    
        // Prepare samples 
        int volume = 1024;
        Audio.prepare(buffer, S, volume);
        
        // Play sound
        Audio.write(buffer, S);
      }
    }
    
    digitalWrite(mutePin, HIGH);
    soundDone = true;
    dimLight = true;

    #ifdef SOFTWAREDEBUG
    //Serial.println("SOUND DONE vv");
    //Serial.println(soundDone);
    #endif
    Serial.println(noDim);
    Serial.println(dimLight);
  }

  if((micros() - timeOfBuzz > 5000000) && dimLight) {
    dimmed = true;
    noDim = false;
  }
  if(dimLight && !noDim) {
    if(redOn) {
      analogWrite(lights[0], sin((micros() - timeOfBuzz + 1200000) / 1200000.0) * 30.0 + 31);
    }
    else {
      analogWrite(lights[9], sin((micros() - timeOfBuzz + 1200000) / 1200000.0) * 30.0 + 31);
    }
  }

  // handles microsecond overflow
  if((micros() - lastSuperClear) > 3600000000) {
    lastSuperClear = micros();
  }
  
  if(dimmed && noDim) {
    Serial.println("I GOT IN HERE");
    if(redOn) {
      analogWrite(lights[0], 60);
      analogWrite(lights[9], 0);
    } else if(greenOn) {
      analogWrite(lights[0], 0);
      analogWrite(lights[9], 60);
    } else {
      analogWrite(lights[0], 0);
      analogWrite(lights[9], 0);
    }
    lightsOff = true;
    dimmed = false;
  }
}



/*DOCUMENTATION/*
GLOBAL VARIABLES
----------------
AUDIO
File bzA: wav file object for red team sound
File bzB: wav file object for green team sound
bool soundDone: true if sound is done playing, false otherwise

PINS
int numInputs: number of buttons, including clear (9 )
int numLights: number of lights, including moderator lights (10)
int buttons[9]: array containing the pins of the 9 buttons, including clear
BUTTONS KEY: [Clear, R1, R2, R3, R4, G1, G2, G3, G4]
int lights[10]: array containing the pins of the 10 lights, including moderator lights
LIGHTS KEY: [RM, R1, R2, R3, R4, G1, G2, G3, G4, GM]

TIMING VARIABLES
unsigned long MAX_VAL: largest possible unsigned long, used as placeholder to sort buzzes
unsigned long SETUP_TIME: stores time at which program began running
unsigned long lastSuperClear: stores time of last super clear (clear in which no button is being held down)
unsigned long bigLastPresses[9]: stores last buzz times for each button in relation to lastSuperClear, including clear (uses BUTTONS KEY)
unsigned long timeOfBuzz: stores the time audio started for current buzz, used for dimming

INTERRUPT VARIABLES
bool clearState: true if clear is currently pressed, false otherwise. used to prevent release of clear from doing anything
unsigned long debounceDelay: minimum time between lastPresses and now for ISR to be reattached (set to 50000 microseconds)
unsigned long interruptDelay: minimum time between when the ISR was last attached and the buzz time for said ISR to fire (set to 50000 microseconds)
bool isAttached[9]: stores whether a given ISR is attached (uses BUTTONS KEY)
bool isRunning[9]: stores whether a given ISR is running (uses BUTTONS KEY)
bool lastAttachTimes[9]: stores last times an ISR was attached in relation to lastSuperClear (uses BUTTONS KEY)
bool hasBuzzed[9]: stores whether each button has buzzed since last super clear (uses BUTTONS KEY)

LOCKOUT SYSTEM
bool redOn: true if red team’s buzz is currently active, false otherwise
bool greenOn: true if green team’s buzz is currently active, false otherwise
bool locked: true if players are unable to buzz, false otherwise
bool dimLight: UNCLEAR PURPOSE, DELETE???
bool isDim: true if light has already been dimmed, false otherwise
bool noDim: true if clear has just happened, false otherwise (used to prevent light from dimming if cleared happens inside of conditional)
bool lightsOff: true if lights are on, false otherwise (used to turn off lights if interrupt fires inside of conditional


LOCAL VARIABLES
----------------

CLEAR
bool isClear: true if no valid buttons are pressed, false otherwise
bool states[9]: stores the states (pressed or unpressed) of each button. true if pressed, false otherwise
unsigned long lastPresses[8]: stores last buzz times for each button in relation to lastSuperClear, not including clear (uses BUTTONS KEY, used for sort)
int lastIndices[8]: stores button indices sorted by press times (ascending)
int newArray[8]: stores intermediate sort values of iteration for placement inside of lastIndices

MISC
unsigned long interruptClear through interruptG4: stores times of buzzes for storage in bigLastPresses


*/
