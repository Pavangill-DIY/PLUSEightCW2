#include <U8g2lib.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>

//Knobs class
//Class declarations
class Knobs
{
  public:
      Knobs(int kt, int ul){
        knobType = kt;
        upper_limit = ul;
        current_pressed = false;
      }
      void update_rotation(){
        if ((past_Knob == 0 && present_Knob == 2)||(past_Knob == 2 && present_Knob == 3)||(past_Knob == 1 && present_Knob == 0)||(past_Knob == 3 && present_Knob == 1)) {
          __atomic_sub_fetch(&knobRotation, 1, __ATOMIC_RELAXED);
          check_plusmin = -1;
        }
        if ((past_Knob == 0 && present_Knob == 1)||(past_Knob == 2 && present_Knob == 0)||(past_Knob == 1 && present_Knob == 3)||(past_Knob == 3 && present_Knob == 2)){
          __atomic_add_fetch(&knobRotation, 1, __ATOMIC_RELAXED);
          check_plusmin = 1;
        }
        if((past_Knob == 0 && present_Knob == 4)||(past_Knob == 4 && present_Knob == 0)||(past_Knob == 1 && present_Knob == 2)||(past_Knob == 2 && present_Knob == 1)){
          //00 to 11, 01 to 10, 10 to 01, 11 to 00 impossible states
          __atomic_add_fetch(&knobRotation, check_plusmin*1, __ATOMIC_RELAXED);
        }
    
        if ( knobRotation > upper_limit){
          __atomic_store_n(&knobRotation,upper_limit,__ATOMIC_RELAXED);
        } 
        if ( knobRotation < 0){
          __atomic_store_n(&knobRotation,0,__ATOMIC_RELAXED);
        }
        past_Knob = present_Knob;
    
        if(current_pressed == true){
          switch(knobType){
            case 0: //metronome
              knobRotation = 32;
              break;
            case 1: //frequency tuning
              knobRotation = 8;
              break;
            case 2: //octave
              knobRotation = 8;
              break;
            case 3: //volume
              knobRotation = 8;
              break;
            default:
              knobRotation = 0;
          }
        }
    }
    
    void setPressed(bool p){
      prev_pressed = current_pressed;
      current_pressed = p;
    }

    void setPresentKnob(int pk){
      past_Knob = present_Knob;
      present_Knob = pk;
    }

    bool knobToggle(){
      return !(prev_pressed && current_pressed);
    }
    
    int getKnobRotation(){
      return knobRotation;
    }

    int getPresentKnob(){
      return present_Knob;
    }
  private:
      int present_Knob;
      int past_Knob;
      int check_plusmin = 0;
      int knobRotation;
      int knobType;
      int threshold;
      int upper_limit;
      bool prev_pressed; 
      bool current_pressed;
};

//Pin definitions

//Global variables
Knobs Knobs3 = Knobs(3,16);
Knobs Knobs2 = Knobs(2,8); ////Declaration of Octave Knob
Knobs Knobs1 = Knobs(1,16); //Declaration of Flat Frequency Knob
Knobs Knobs0 = Knobs(0,120); //Declaration of Metronome Knob

int previous_allkeypressed[12];
int allkeypressed[12]; 
int InMsg_allkeypressed[12]; 
int Serial_control;
SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t all_key_mutex;
SemaphoreHandle_t Knob0Mutex;
SemaphoreHandle_t Knob1Mutex;
SemaphoreHandle_t Knob2Mutex;
SemaphoreHandle_t Knob3Mutex;


volatile uint32_t currentStepSize0;
volatile uint32_t currentStepSize1;
volatile uint32_t currentStepSize2;
volatile uint32_t currentStepSize3;
volatile uint32_t currentStepSize4;
volatile uint32_t currentStepSize5;
volatile uint32_t currentStepSize6;
volatile uint32_t currentStepSize7;
volatile uint32_t currentStepSize8;
volatile uint32_t currentStepSize9;
volatile uint32_t currentStepSize10;
volatile uint32_t currentStepSize11;

volatile uint32_t currentStepSizeArray[12];
volatile uint8_t keyArray[7];
volatile int8_t knob3Rotation;
volatile int8_t GLOBknob3Rotation;
volatile int8_t knob2Rotation;
volatile int8_t knob1Rotation;
volatile int8_t knob0Rotation;

//Time elapsed for metronome
unsigned long mTime1 = millis();
unsigned long mTime2 = millis();
bool met_select = false;
bool blinkLight = false;

volatile char noteMessage[] = "xxx";
QueueHandle_t msgOutQ;
//Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

//Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

//Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

//Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;
bool JOY_PIN_PREV = false;
bool JOY_PIN_CURRENT = false;
bool JOY_PIN_PRESSED = false;

bool joyToggle(){
  return !(JOY_PIN_PREV && JOY_PIN_CURRENT);
}
//Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

//Constants
int octave;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Lookup tables
const char intToHex[] = "0123456789ABCDEF";
const String Notes[] =
{
  "C",
  "C#",
  "D",
  "D#",
  "E",
  "F",
  "F#",
  "G",
  "G#",
  "A",
  "A#",
  "B",
};
int ASCIIHexToInt[] =
{
    // ASCII
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, -1, -1, -1, -1, -1, -1,
    -1, 10, 11, 12, 13, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, 10, 11, 12, 13, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    // 0x80-FF (Omit this if you don't need to check for non-ASCII)
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
};

int Octavetochar[]{
  '0',
  '1',
  '2',
  '3',
  '4',
  '5',
  '6',
  '7',
  '8'
};

const uint32_t stepSizes[]= {
51076922.44,
54112683.41,
57330004.37,
60740598.85,
64352275.9,
68178701.31,
72231588.63,
76528508.18,
81077269,
85899345.92,
91006452.49,
96418111.28,
};

//Function to set outputs via matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

void setup() {
  // put your setup code here, to run once:
  knob3Rotation = 0;//set rotation to zero
  //setup message queue. 8 items, size of each item
  msgOutQ = xQueueCreate(8, 4);
  
  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(115200);

  //Task handling
  TaskHandle_t msgOutTaskHandle = NULL;
  xTaskCreate(
    msgOutTask,
    "msgOut",
    32,
    NULL,
    3,
    &msgOutTaskHandle);
    
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,
    "scanKeys",
    32,
    NULL,
    4,
    &scanKeysHandle);
    
  TaskHandle_t displayUpdateTaskHandle = NULL;
  xTaskCreate(
    displayUpdateTask,
    "displayUpdate",
    256,
    NULL,
    2,
    &displayUpdateTaskHandle);

   TaskHandle_t msgToBoardHandle = NULL;
  xTaskCreate(
   msgToBoard,
  "msgToBoard",
   32,
   NULL,
    1,
  &msgToBoardHandle);
  
  keyArrayMutex = xSemaphoreCreateMutex();
  all_key_mutex = xSemaphoreCreateMutex();
  Knob0Mutex = xSemaphoreCreateMutex();
  Knob1Mutex = xSemaphoreCreateMutex();
  Knob2Mutex = xSemaphoreCreateMutex();
  Knob3Mutex = xSemaphoreCreateMutex();

  vTaskStartScheduler();
  
}


uint8_t readCols(){
  int C0 = (digitalRead (C0_PIN));
  int C1 = (digitalRead (C1_PIN))*2;
  int C2 = (digitalRead (C2_PIN))*4;
  int C3 = (digitalRead (C3_PIN))*8;
  int Column = C3 + C2 + C1 + C0;
  return (Column);
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN,HIGH);
}

//For setting localCurrentStepSize, using flat frequency from frequency tuning and octave knob rotations. Note: joy pin off
int tuneFrequency(int localCurrStepSize, int knob1Rotation, int knob2Rotation)
{
  int localCurrentStepSize = localCurrStepSize;

  switch (knob1Rotation)
  {
  case (0):
    localCurrentStepSize = localCurrStepSize - (800000 * knob2Rotation);
    break;
  case (1):
    localCurrentStepSize = localCurrStepSize - (700000 * knob2Rotation);
    break;
  case (2):
    localCurrentStepSize = localCurrStepSize - (600000 * knob2Rotation);
    break;
  case (3):
    localCurrentStepSize = localCurrStepSize - (500000 * knob2Rotation);
    break;
  case (4):
    localCurrentStepSize = localCurrStepSize - (400000 * knob2Rotation);
    break;
  case (5):
    localCurrentStepSize = localCurrStepSize - (300000 * knob2Rotation);
    break;
  case (6):
    localCurrentStepSize = localCurrStepSize - (200000 * knob2Rotation);
    break;
  case (7):
    localCurrentStepSize = localCurrStepSize - (100000 * knob2Rotation);
    break;
  case (8):
    localCurrentStepSize = localCurrStepSize;
    break;
  case (9):
    localCurrentStepSize = localCurrStepSize + (100000 * knob2Rotation);
    break;
  case (10):
    localCurrentStepSize = localCurrStepSize + (200000 * knob2Rotation);
    break;
  case (11):
    localCurrentStepSize = localCurrStepSize + (300000 * knob2Rotation);
    break;
  case (12):
    localCurrentStepSize = localCurrStepSize + (400000 * knob2Rotation);
    break;
  case (13):
    localCurrentStepSize = localCurrStepSize + (500000 * knob2Rotation);
    break;
  case (14):
    localCurrentStepSize = localCurrStepSize + (600000 * knob2Rotation);
    break;
  case (15):
    localCurrentStepSize = localCurrStepSize + (700000 * knob2Rotation);
    break;
  case (16):
    localCurrentStepSize = localCurrStepSize + (800000 * knob2Rotation);
    break;
  default:
    break;
  }
  if (knob2Rotation == 0)
  {
    localCurrentStepSize = 0;
  }

  return localCurrentStepSize;
}

void sampleISR() {
  static uint32_t phaseAcc[12]= {0}; //define static memory phase accumulator so it stays the same for each function call
  uint32_t outValue[12]= {0}; // arrays of 12, keep track of outvalue and phaseacc for each key 
  uint32_t outaverageValue=0; //final output sound
  int count = 0; //keeps track of number of keys pressed
  uint32_t LocalcurrentStepSizeArray[12];
  int8_t Localknob3Rotation;

  //extract currentstepsize back into a local array
  LocalcurrentStepSizeArray[0] = currentStepSize0;
  LocalcurrentStepSizeArray[1] = currentStepSize1;
  LocalcurrentStepSizeArray[2] = currentStepSize2;
  LocalcurrentStepSizeArray[3] = currentStepSize3;
  LocalcurrentStepSizeArray[4] = currentStepSize4;
  LocalcurrentStepSizeArray[5] = currentStepSize5;
  LocalcurrentStepSizeArray[6] = currentStepSize6;
  LocalcurrentStepSizeArray[7] = currentStepSize7;
  LocalcurrentStepSizeArray[8] = currentStepSize8;
  LocalcurrentStepSizeArray[9] = currentStepSize9;
  LocalcurrentStepSizeArray[10] = currentStepSize10;
  LocalcurrentStepSizeArray[11] = currentStepSize11;
  LocalcurrentStepSizeArray[11] = currentStepSize11;
  Localknob3Rotation = GLOBknob3Rotation;


  for(int i=0 ; i<12; i++){
    phaseAcc[i]+= LocalcurrentStepSizeArray[i]; 
    if(LocalcurrentStepSizeArray[i]==0){
      phaseAcc[i]=0; //reset phase acc per key if that key is not pressed
    }
    outValue[i]= (phaseAcc[i] >> 24) >> (8 - Localknob3Rotation/2); //first 8 bits then adjusted for sound knob
    if (outValue[i]>0){
      count++; //number of audio sources
    }
  }
  for(int i = 0 ; i <12; i++){
    outaverageValue += outValue[i]; 
  }
  outaverageValue = outaverageValue/count;
  if(outaverageValue > 255){
    outaverageValue = 254; //adjust for any clipping values
  }
  analogWrite(OUTR_PIN, outaverageValue);
}


  
void scanKeysTask(void * pvParameters){
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  bool oscillation = false;
  int localCounter = 10;
  int localCurrentStepSize;
  int localOctave;
  int scankeysKnobRotation;
  
  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    if (keyArray[5] == 11 && met_select == false){
      JOY_PIN_PREV = JOY_PIN_CURRENT;
      JOY_PIN_CURRENT = true;
      if(joyToggle()){
        JOY_PIN_PRESSED = !JOY_PIN_PRESSED;
      }
    } else {
        JOY_PIN_PREV = JOY_PIN_CURRENT;
        JOY_PIN_CURRENT = false;
    }
    
    for(int i = 0; i <=7; i++){
      setRow(i);
      delayMicroseconds(3);
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      keyArray[i] = readCols();
      xSemaphoreGive(keyArrayMutex);
    }
    int i = 0;
    int j = 0;
    int Select = 0;
    int Akeypressed[4];
    int localCurrentStepSizeArray[12];
    int localallkeypressed[12];
    noteMessage[1] = Octavetochar[octave];


    for(int i = 0; i <=2; i++){
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      int key_array_check = keyArray[i];
      xSemaphoreGive(keyArrayMutex);

      if (key_array_check ==  0xF){//c // key0
        Akeypressed[0]=0; //first key
        Akeypressed[1]=0; //2nd key
        Akeypressed[2]=0; //third key
        Akeypressed[3]=0; //fourth key c0 active low so keypressed   
      }
      if (key_array_check ==  0xE){//c // key0
        Akeypressed[0]=1; //first key
        Akeypressed[1]=0; //2nd key
        Akeypressed[2]=0; //third key
        Akeypressed[3]=0; //fourth key c0 active low so keypressed
      }
      if (key_array_check ==  0xD){//C# //key1
        Akeypressed[0]=0;
        Akeypressed[1]=1;
        Akeypressed[2]=0;
        Akeypressed[3]=0; 
      }
      if (key_array_check ==  0xC){//C+C# //key 0 1
        Akeypressed[0]=1;
        Akeypressed[1]=1;
        Akeypressed[2]=0;
        Akeypressed[3]=0;
      }
      if (key_array_check ==  0xB){//D //key 2
        Akeypressed[0]=0;
        Akeypressed[1]=0;
        Akeypressed[2]=1;
        Akeypressed[3]=0;
      }
      if (key_array_check ==  0xA){//D+c // key2 0
        Akeypressed[0]=1;
        Akeypressed[1]=0;
        Akeypressed[2]=1;
        Akeypressed[3]=0;
      }
      if (key_array_check ==  0x9){//C#+D //key 1 2
        Akeypressed[0]=0;
        Akeypressed[1]=1;
        Akeypressed[2]=1;
        Akeypressed[3]=0; 
      }
      if (key_array_check ==  0x8){//C+C#+D //0 1 2
        Akeypressed[0]=1;
        Akeypressed[1]=1;
        Akeypressed[2]=1;
        Akeypressed[3]=0;
      }
     if (key_array_check ==  0x7){//D# // 3
        Akeypressed[0]=0;
        Akeypressed[1]=0;
        Akeypressed[2]=0;
        Akeypressed[3]=1;
      }
      if (key_array_check ==  0x6){//D#+C //3 0
        Akeypressed[0]=1;
        Akeypressed[1]=0;
        Akeypressed[2]=0;
        Akeypressed[3]=1;
      }
      if (key_array_check ==  0x5){//D#+C# // 3 1
        Akeypressed[0]=0;
        Akeypressed[1]=1;
        Akeypressed[2]=0;
        Akeypressed[3]=1;
      }
      if (key_array_check ==  0x4){//D#+C#+C //3 1 0
        Akeypressed[0]=1;
        Akeypressed[1]=1;
        Akeypressed[2]=0;
        Akeypressed[3]=1;
      }
      if (key_array_check ==  0x3){//D#+D //3 2
        Akeypressed[0]=0;
        Akeypressed[1]=0;
        Akeypressed[2]=1;
        Akeypressed[3]=1;
      }
      if (key_array_check ==  0x2){//D#+D+C // 3 2 0
        Akeypressed[0]=1;
        Akeypressed[1]=0;
        Akeypressed[2]=1;
        Akeypressed[3]=1;        
      }
      if (key_array_check ==  0x1){//D#+D+C# // 3 2 1
        Akeypressed[0]=0;
        Akeypressed[1]=1;
        Akeypressed[2]=1;
        Akeypressed[3]=1;
      }
      if (key_array_check ==  0x0){//All
        Akeypressed[0]=1;
        Akeypressed[1]=1;
        Akeypressed[2]=1;
        Akeypressed[3]=1;        
      }
      for(int k=0; k<4;k++){
          localallkeypressed[((i*4)+k)] = Akeypressed[k];
      }
   }
  //toggle loop
  if (Serial_control == 0){
    xSemaphoreTake(all_key_mutex, portMAX_DELAY);
    for(int k=0; k<12;k++){
          allkeypressed[k] = localallkeypressed[k];
    }
    xSemaphoreGive(all_key_mutex);
  }
  if (Serial_control == 1){
    knob2Rotation = octave;
  }
   
    for(int j= 0; j <12; j++){
      if (allkeypressed[j] == 1){
        localCurrentStepSize = stepSizes[j];
        //setting octave
        octave = knob2Rotation;
        localOctave = octave;

        ///////////////////////////////////( ▀ ͜͞ʖ▀) ///////////////////////////////////
        //WHAM STICK/
        ////////rock on dude//////////////////////////////////////////////////////////
        
        if (oscillation  == false && JOY_PIN_PRESSED == true && met_select == false){
          if (octave > 4){
          localOctave = (localOctave - 4)*2;
          localCurrentStepSize = localCurrentStepSize*localOctave;}
          if (octave < 4){
          localOctave = (4-localOctave)*2;
          localCurrentStepSize = localCurrentStepSize/localOctave;
        }
        localCurrentStepSize = ((analogRead(A0)/64*localCurrentStepSize/localCounter+10000000)) ;
        localCounter++;  
        if (localCounter==40){
        oscillation = true;
        }
        }
        else if (oscillation  == true && JOY_PIN_PRESSED == true && met_select == false){
        if (octave > 4){
          localOctave = (localOctave - 4)*2;
          localCurrentStepSize = localCurrentStepSize*localOctave;}
           if (octave < 4){
          localOctave = (4-localOctave)*2;
          localCurrentStepSize = localCurrentStepSize/localOctave;
        }
        localCurrentStepSize  = ((analogRead(A0)/64*localCurrentStepSize/localCounter+10000000)); 
        localCounter--;
        if (localCounter==10){
        oscillation = false;
        }
        }
        ////////rock off dude//////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////

        // Setting Octave
        if (octave > 4 && JOY_PIN_PRESSED == false){
          localOctave = (localOctave - 4)*2;
          localCurrentStepSize = localCurrentStepSize*localOctave;
        }
        if (octave < 4 && JOY_PIN_PRESSED == false){
          localOctave = (localOctave - 4)*2;
          localCurrentStepSize = localCurrentStepSize/localOctave;
        }
        
        //Setting flat frequency
        localCurrentStepSize = tuneFrequency(localCurrentStepSize, knob1Rotation, knob2Rotation);
        
      } 
      else{
        localCurrentStepSize = 0;
      }
      localCurrentStepSizeArray[j] = localCurrentStepSize;
    }
    
    //If Metronome mode is selected [deliberately separate from key scanning]
    mTime1 = millis(); //update current time
    if(met_select == true && JOY_PIN_PRESSED == false){
      int ms_per_beat = 60000/(40+knob0Rotation*3); //standard range of 40 - 400
      if((mTime1 - mTime2) > ms_per_beat) {
        blinkLight = true;
        //pass the metronome sound
        for(int k = 0; k<12; k++){
            currentStepSizeArray[k] = 31076922;//10000000;
        }
        mTime2 = millis();
      } 
      else{
        for(int  k = 0; k<12; k++){
            //Make keyboard unavailable
            currentStepSizeArray[k] = 0;
        }
      }
    }
    else{
      blinkLight = false;
      for(int k = 0; k<12; k++){
        currentStepSizeArray[k] = localCurrentStepSizeArray[k];
      }      
    }

    ///////////////////////////////////////////////////////////////////////////////
    // KNOB UPDATES
    ///////////////////////////////////////////////////////////////////////////////
    
    //Volume knob
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    int temp5 = keyArray[5];
    int temp6 = keyArray[6];
    int temp3 = keyArray[3];
    int temp4 = keyArray[4];
    xSemaphoreGive(keyArrayMutex);

    xSemaphoreTake(Knob3Mutex, portMAX_DELAY);
    Knobs3.setPresentKnob(temp3%4);
    if (temp5 == 13){
      Knobs3.setPressed(true);
    } 
    else{
      Knobs3.setPressed(false); //reset button press
    }
    if (JOY_PIN_PRESSED == true){
        knob3Rotation = 8 + analogRead(A1)/128;
    }
    else {
      knob3Rotation = Knobs3.getKnobRotation();
    }
    Knobs3.update_rotation();
    int localknobRotation = knob3Rotation;
    xSemaphoreGive(Knob3Mutex);

    //Octave Knob update
    xSemaphoreTake(Knob2Mutex, portMAX_DELAY);
    Knobs2.setPresentKnob(temp3/4);
    if (temp5 == 14){
      Knobs2.setPressed(true);
    }
    else{
      Knobs2.setPressed(false);
    }
    knob2Rotation = Knobs2.getKnobRotation();
    Knobs2.update_rotation();
    xSemaphoreGive(Knob2Mutex);

    //Flat Frequency Knob update
    xSemaphoreTake(Knob1Mutex, portMAX_DELAY);
    Knobs1.setPresentKnob(temp4%4);
    if (temp6 == 13){
      Knobs1.setPressed(true);
    } 
    else{
      Knobs1.setPressed(false);
    }
    knob1Rotation = (Knobs1.getKnobRotation());
    Knobs1.update_rotation();
    xSemaphoreGive(Knob1Mutex);

    //Metronome Knob update
    xSemaphoreTake(Knob0Mutex, portMAX_DELAY);
    Knobs0.setPresentKnob(temp4/4);
    if (temp6 == 14 && JOY_PIN_PRESSED == false){
      Knobs0.setPressed(true);
      if(Knobs0.knobToggle()){
        met_select = !met_select;
      }
    }else{
      Knobs0.setPressed(false);
    } 
    knob0Rotation = Knobs0.getKnobRotation();
    Knobs0.update_rotation();
    xSemaphoreGive(Knob0Mutex);

    ///////////////////////////////////////////////////////////////////////////////
    // KEY OUTPUTS
    ///////////////////////////////////////////////////////////////////////////////

    //To Serial print
    for( int k = 0; k<12; k++){
      xSemaphoreTake(all_key_mutex, portMAX_DELAY);
        if (previous_allkeypressed[k] == 0 && allkeypressed[k] == 1){
          noteMessage[0] = 'P';
          noteMessage[2] = intToHex[k];
          xQueueSend(msgOutQ, (char*) noteMessage, portMAX_DELAY);
        }
        if (previous_allkeypressed[k] == 1 && allkeypressed[k] == 0){
          noteMessage[0] = 'R';
          noteMessage[2] = intToHex[k];
          xQueueSend(msgOutQ, (char*) noteMessage, portMAX_DELAY);
        }  
      xSemaphoreGive(all_key_mutex);
    }
    
    // Store previous stepsize 
    xSemaphoreTake(all_key_mutex, portMAX_DELAY);
    for( int k=0; k<12; k++){
      previous_allkeypressed[k] = allkeypressed[k];
    }
    xSemaphoreGive(all_key_mutex);
    
  
    //convert the current to atomic to be sent to ISR
    __atomic_store_n(&currentStepSize0,currentStepSizeArray[0],__ATOMIC_RELAXED);
    __atomic_store_n(&currentStepSize1,currentStepSizeArray[1],__ATOMIC_RELAXED);
    __atomic_store_n(&currentStepSize2,currentStepSizeArray[2],__ATOMIC_RELAXED);
    __atomic_store_n(&currentStepSize3,currentStepSizeArray[3],__ATOMIC_RELAXED);
    __atomic_store_n(&currentStepSize4,currentStepSizeArray[4],__ATOMIC_RELAXED);
    __atomic_store_n(&currentStepSize5,currentStepSizeArray[5],__ATOMIC_RELAXED);
    __atomic_store_n(&currentStepSize6,currentStepSizeArray[6],__ATOMIC_RELAXED);
    __atomic_store_n(&currentStepSize7,currentStepSizeArray[7],__ATOMIC_RELAXED);
    __atomic_store_n(&currentStepSize8,currentStepSizeArray[8],__ATOMIC_RELAXED);
    __atomic_store_n(&currentStepSize9,currentStepSizeArray[9],__ATOMIC_RELAXED);
    __atomic_store_n(&currentStepSize10,currentStepSizeArray[10],__ATOMIC_RELAXED);
    __atomic_store_n(&currentStepSize11,currentStepSizeArray[11],__ATOMIC_RELAXED);
    __atomic_store_n(&GLOBknob3Rotation,localknobRotation,__ATOMIC_RELAXED);


  }
}

void msgOutTask(void *pvParameters) {
    const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();

     char outMsg[4];
     while (1) {
       while(xQueueReceive(msgOutQ, outMsg, 0) != pdTRUE){
         vTaskDelay(10);
        };
       Serial.println(outMsg);
     }
}

void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.setCursor(2,10);
    u8g2.print("PlusEight");
    u8g2.setCursor(64,10);
    //add for loop to update the note message for sending it 
    u8g2.print((char*) noteMessage);
    u8g2.setCursor(2,20);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    u8g2.print(keyArray[0],HEX);
    u8g2.print(keyArray[1],HEX);
    u8g2.print(keyArray[2],HEX);
    xSemaphoreGive(keyArrayMutex);

    u8g2.setCursor(50,20);
    //BPM conversion
    if(met_select){
      xSemaphoreTake(Knob0Mutex, portMAX_DELAY);
      u8g2.print((40+knob0Rotation*3),DEC);
      xSemaphoreGive(Knob0Mutex);
      u8g2.print("bpm"); 
    } else{
      u8g2.print(0,DEC);
      u8g2.print("bpm"); 
    }

    
    u8g2.setCursor(100,20);
    u8g2.print(Notes[ASCIIHexToInt[noteMessage[2]]]);
    u8g2.print(octave,DEC);

    //display knobs
    u8g2.setCursor(2,30);
    u8g2.print("FLAT:");
    
    xSemaphoreTake(Knob1Mutex, portMAX_DELAY);
    u8g2.print(knob1Rotation,DEC);
    xSemaphoreGive(Knob1Mutex);

    u8g2.setCursor(50,30);
    u8g2.print("OCT:");
    
    xSemaphoreTake(Knob2Mutex, portMAX_DELAY);
    u8g2.print(knob2Rotation,DEC);
    xSemaphoreGive(Knob2Mutex);
    u8g2.setCursor(90,30);
    u8g2.print("VOL:");
   
    xSemaphoreTake(Knob3Mutex, portMAX_DELAY);
    u8g2.print(knob3Rotation,DEC);
    xSemaphoreGive(Knob3Mutex);

    u8g2.sendBuffer();          // transfer internal memory to the display
    //Toggle LED
    if(blinkLight){
      digitalWrite(LED_BUILTIN, HIGH);
    } else{
      digitalWrite(LED_BUILTIN, LOW);
    }

  }
}

void msgToBoard(void * pvParameters){
  const TickType_t xFrequency = 10/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  char inMsg[] = "xxx";
  int localKey;

  while (1){
    delay(500);
    for (int i = 0; i <= 2; i ++){
      inMsg[i] = Serial.read();
        }
    
    if (ASCIIHexToInt[inMsg[2]] >=0){
      localKey = ASCIIHexToInt[inMsg[2]];
    }
     if (ASCIIHexToInt[inMsg[1]] >=0){
      octave = ASCIIHexToInt[inMsg[1]];
    }
    if (inMsg[0] == 'P'){
        InMsg_allkeypressed[localKey] = 1;
        noteMessage[0]= 'P';
       }
    if (inMsg[0] == 'R'){
        InMsg_allkeypressed[localKey] = 0;
        noteMessage[0]= 'R';
       }
    xSemaphoreTake(all_key_mutex, portMAX_DELAY);
    for(int k=0; k<12; k++){
      if (InMsg_allkeypressed[k] == 0||InMsg_allkeypressed[k] == 1){
        allkeypressed[k] = InMsg_allkeypressed[k];
      }
    }
    xSemaphoreGive(all_key_mutex);

     for(int k=0; k<12; k++){
      Serial_control = 0;
      if (InMsg_allkeypressed[k] == 1){
        Serial_control = 1;
        k = 13;
      }
    }
    
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  //Update display
}
