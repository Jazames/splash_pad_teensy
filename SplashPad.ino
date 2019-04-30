//Header Includes
#include "Adafruit_TLC5947.h"
#define BLE_serial Serial1
#define DELAY_TIME 5000
#define Detector_serial Serial2

//Global Variables
uint8_t BLE_buffer[98] = //Puts all nozzles in object tracker mode, and sorta turns the color to whiteish. 
{
  0xFF, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x08, 0x3D, 0x06, 0x33, 0x0B, 0x10,
  0xFF, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x08, 0x3D, 0x06, 0x33, 0x0B, 0x10,
  0xFF, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x08, 0x3D, 0x06, 0x33, 0x0B, 0x10,
  0xFF, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x08, 0x3D, 0x06, 0x33, 0x0B, 0x10,
  0xFF, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x08, 0x3D, 0x06, 0x33, 0x0B, 0x10,
  0xFF, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x08, 0x3D, 0x06, 0x33, 0x0B, 0x10,
  0xFF, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x08, 0x3D, 0x06, 0x33, 0x0B, 0x10
}
uint32_t nozzle_state[7] = {1, 1, 1, 0, 1, 1, 1};
uint32_t object_tracker_nozzle_modes[7] = {0, 0, 0, 0, 0, 0, 0}; //All on, so that if it works, they'll turn off. 
Adafruit_TLC5947 led_driver(1, 13, 11, 10);
//Spi Pins
  //CS 10
  //DOUT 11
  //SCK 13

//Functions
bool parse_BLE_message(uint8_t);
void interpret_BLE_message();
bool read_BLE(uint8_t*);
static void set_color(uint16_t, uint8_t, uint8_t*);
uint16_t get_color(uint8_t, uint8_t*);
void check_pressure();
void update_nozzles();
void write_colors();
void randomFunction(int);

uint32_t getObjectTrackingSuggestion(int nozzle);

//Communication with Object Detector
void writeToObjectDetector(uint8_t message_type, uint8_t* buffer, int payload_length)
uint16_t calculateFletcher16(uint8_t* data, int length);
void insertChecksum(uint8_t* buffer, int length);
void runCommunicator();
void processSerialInput();
void parseSerial(uint8_t byte);
void interpretMessage(uint8_t* buffer);
void interpretCalibrationMessage(uint8_t* buffer);
void interpretSetColorMessage(uint8_t* buffer);
void interpretSetNozzleMessage(uint8_t* buffer);
void sendCalibrationMessage();






//Const definitions
#define DesiredPressure 1350 //Desired Value
#define PressurePin 23 //pin number
#define OpenValve 22 //pin number
#define CloseValve 21 //pin number
const int Nozzle[7] = {14, 15, 16, 17, 18, 19, 20};//pin numbers

#define MESSAGE_TYPE_SET_NOZZLE  0X01
#define MESSAGE_TYPE_SET_COLOR   0x02
#define MESSAGE_TYPE_CALIBRATION 0x03

#define START_CALIBRATION_COMMAND 0x01
#define CALIBRATION_COMPLETE_COMMAND 0x02

#define MODE_ON        0x00000001
#define MODE_OFF       0x00000000
#define MODE_RANDOM    0xFF000002
#define MODE_CALIBRATE 0xFF000000
#define MODE_OB_TRACK  0xFF000001

//Uart1
  //RX1 0
  //TX1 1
//Uart3
  //RX3 7
  //TX3 8

void setup() {  
  BLE_serial.begin(115200);
  Serial.begin(115200);
  led_driver.begin();
  Detector_serial.begin(115200); 
//  analogReadResolution(12); //not declared in this scope
  pinMode(22, OUTPUT);  //Open
  pinMode(OpenValve,  OUTPUT);
  pinMode(CloseValve, OUTPUT);
  randomSeed(millis()%3501);
 
  //Setup nozzleys. 
  for(int nozzleNum = 0; nozzleNum < 7; nozzleNum++)
  {
   pinMode(Nozzle[nozzleNum], OUTPUT);
   digitalWrite(Nozzle[nozzleNum], 1);
  }


}

void loop() {
  //ti receive
  while(BLE_serial.available() > 0)
  {
    if(parse_BLE_message(BLE_serial.read()))
    {
      interpret_BLE_message();//also updates nozzles and colors
    }
  }
  while(Detector_serial.available() > 0)
  {
    parseDetectorSerial(Detector_serial.read()); 
  }
  //pi receive
  check_pressure();  //check_pressure
  
}

bool parse_BLE_message(uint8_t current_byte)
{
  static uint8_t state = 0;
  static uint8_t index = 0;
  switch (state)
  {
    case 0:
      if (current_byte == 0xF0)
      {
        state = 1;
      }
      break;
    case 1:
      if (current_byte == 0x01)
      {
        state = 2;
        index = 0;
      }
      else
      {
        state = 0;
      }
      break;
    case 2:
      BLE_buffer[index++] = current_byte;
      if (index >= 97)
      {
        state = 0;
        return true;
      }
      break;
    default:
      state = 0;
  }
  return false;
}

void interpret_BLE_message()
{
  //4bytes mode
  //4bytes duration
  //2bytes red
  //2bytes green
  //2bytes blue
  Serial.println("updated");
  int j = 0;
  uint16_t red;
  uint16_t green;
  uint16_t blue;
  int nozzles_ready_for_calibration = 0;
  for(int i = 0; i < 7 * 14; i+=14)
  {
    uint32_t mode     = (BLE_buffer[i +  0] << 24) | (BLE_buffer[i +  1] << 16) | (BLE_buffer[i + 2] << 8) | (BLE_buffer[i + 3]);
    uint32_t duration = (BLE_buffer[i +  4] << 24) | (BLE_buffer[i +  5] << 16) | (BLE_buffer[i + 6] << 8) | (BLE_buffer[i + 7]);
    red               = (BLE_buffer[i +  8] <<  8) | (BLE_buffer[i +  9]);
    green             = (BLE_buffer[i + 10] <<  8) | (BLE_buffer[i + 11]);
    blue              = (BLE_buffer[i + 12] <<  8) | (BLE_buffer[i + 13]);
    int tag = 3;
    switch(mode)
    {
      case MODE_ON:
        nozzle_state[j] = 0;
        break;
      case MODE_OFF:
        nozzle_state[j] = 1;
        break;
      case MODE_RANDOM:
        randomFunction(j);
        break;
      case MODE_CALIBRATE:
        //uuuuuuuuuuuuuUUUUUUUUUU
        nozzle_state[j] = getObjectTrackingSuggestion(j);
        break;
      case MODE_OB_TRACK:
        nozzle_state[j] = getObjectTrackingSuggestion(j);
        break;
      default:
        nozzle_state[j] = 1;
        break;
    }
    if(mode == MODE_CALIBRATE)
    {
      nozzles_ready_for_calibration++;
    }

    led_driver.setLED(j, red, green, blue);
    j++; //Needs to be last
  }
  if(nozzles_ready_for_calibration == 7)
  {
    sendCalibrationMessage();
  }
  //Serial.println(nozzle_state[j]);   
  led_driver.setLED(7, red, green, blue); //for testing purposes only

  update_nozzles();
  delay(100); //ensures the LED's update properly
  led_driver.write();
  
}

uint16_t get_color(uint8_t position, uint8_t* color_array)
{
    // Byte index and color data visual reference:
    //  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14...
    // 00 01 11 22 23 33 44 45 55 66 67 77 88 89 99...
    if((position & 0x01) == 0x01)//Check if odd
    {
        //Case position is odd
        //color_array[((position - 1) * 3 / 2) + 1] = (color_array[position] & 0xF0) | (color >> 8) & 0x0F;
        //color_array[((position + 1) * 3 / 2) - 1] = color & 0xFF;
        return ((color_array[((position - 1) * 3 / 2) + 1] & 0x0F) << 8) | (color_array[((position + 1) * 3 / 2) - 1]);
    }
    else
    {
        //Case position is even
        //color_array[(position * 3 / 2)] = color >> 4 & 0xFF;
        //color_array[(position * 3 / 2) + 1] = (color_array[(position * 3 / 2) + 1] & 0x0F) | ((color << 4) & 0xF0);
        return (color_array[(position * 3 / 2)] << 4) | ((color_array[(position * 3 / 2) + 1] >> 4) & 0x0F);
    }
}

static void set_color(uint16_t color, uint8_t position, uint8_t* color_array)
{
    // Byte index and color data visual reference:
    //  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14...
    // 00 01 11 22 23 33 44 45 55 66 67 77 88 89 99...
    if((position & 0x01) == 0x01)//Check if odd
    {
        //Case position is odd
        color_array[((position - 1) * 3 / 2) + 1] = (color_array[position] & 0xF0) | (color >> 8) & 0x0F;
        color_array[((position + 1) * 3 / 2) - 1] = color & 0xFF;
    }
    else
    {
        //Case position is even
        color_array[(position * 3 / 2)] = color >> 4 & 0xFF;
        color_array[(position * 3 / 2) + 1] = (color_array[(position * 3 / 2) + 1] & 0x0F) | ((color << 4) & 0xF0);

    }
}

void check_pressure()
{
  int pressure = analogRead(PressurePin);
  if(pressure >= DesiredPressure + 10){
    digitalWrite(CloseValve, 0);
    digitalWrite(OpenValve, 1);
  }
  else if(pressure <= DesiredPressure - 10){
    digitalWrite(OpenValve, 0);
    digitalWrite(CloseValve, 1);
  }
  else{
    digitalWrite(CloseValve, 0);
    digitalWrite(OpenValve, 0);
  }
  Serial.println(pressure);
}

void update_nozzles()
{
  for(int nozzleNum = 0; nozzleNum < 7; nozzleNum++)
  {
    digitalWrite(Nozzle[nozzleNum], nozzle_state[nozzleNum]);
  }
}


void randomFunction(int nozzle_num)
{
    static int last_time = millis();
    int current_time = millis();
    if(current_time - last_time > DELAY_TIME) 
    {
      last_time = current_time + (current_time - last_time);
      int state = random(0, 99);
      if(state < 50)
      {
        nozzle_state[nozzle_num] = 0;
      }
      else
      {
        nozzle_state[nozzle_num] = 1;
      }
    } 
}

uint32_t getObjectTrackingSuggestion(int nozzle)
{
  return object_tracker_nozzle_modes[nozzle]; ///Yeah, this function does nothing. That's okay though. 
}



/////////////////////////////////
//OBJECT DETECTOR COMMUNICATION//
/////////////////////////////////
void writeToObjectDetector(uint8_t message_type, uint8_t* buffer, int payload_length)
{
  //Write out the nozzle buffer. 
  int length = payload_length + 5;
  static uint8_t message[50]; //Need to update this if it gets too smol. 
  message[0] = 0xF0;
  message[1] = 0x0D;
  message[2] = message_type;

  //Because this is easier than looking up how to use mem copy
  for(int i = 0; i < payload_length; i++)
  {
    message[i + 3] = buffer[i];
  }
  insertChecksum(message, length - 2);
  //write(mUARTHandle, message, length);

  Detector_serial.write(message, length);
}

uint16_t calculateFletcher16(uint8_t* data, int length)
{
   uint16_t sum1 = 0;
   uint16_t sum2 = 0;
   int index;

   for(index = 0; index < length; index++)
   {
      sum1 = (sum1 + data[index]) % 0xFF;
      sum2 = (sum2 + sum1) % 0xFF;
   }

   return (sum2 << 8) | sum1;
}

void insertChecksum(uint8_t* buffer, int length)
{
  uint16_t checksum = calculateFletcher16(buffer, length);
  buffer[length] = (checksum >> 8) & 0xFF;
  buffer[length + 1] = (checksum)  & 0xFF;
}



void parseDetectorSerial(uint8_t byte)
{
  static int incoming_state = 0;
  static int incoming_index = 0;
  static uint8_t incoming_buffer[100];
  static uint8_t incoming_type = 0x00; 
  static uint16_t incoming_checksum = 0;


  switch(incoming_state)
  {
    case 0:
      if(byte == 0xF0)
      {
        incoming_state = 1;
        incoming_index = 0;
        incoming_buffer[incoming_index] = byte;
        incoming_index++;
      }
      break;
    case 1: 
      if(byte == 0x0D)
      {
        incoming_state = 2;
        incoming_index = 0;
      }
      else
      {
        incoming_state = 0;
        incoming_buffer[incoming_index] = byte;
        incoming_index++;
      }
      break;
    case 2:
      incoming_type = byte;
      incoming_buffer[incoming_index] = byte;
      incoming_index++;
      incoming_state = 3;
      break;
    case 3:
      incoming_buffer[incoming_index] = byte;
      incoming_index++;
      if(incoming_index >= getMessageLength(incoming_type) - 1)
      {
        incoming_state = 4;
      }
      break;
    case 4: 
      incoming_checksum = ((uint16_t)byte << 8);
      break;
    case 5: 
      incoming_checksum = incoming_checksum | byte;
      uint16_t calcChecksum = calculateFletcher16(incoming_buffer, getMessageLength(incoming_type))
      if(calcChecksum == incoming_checksum)
      {
        interpretMessage(incoming_buffer); 
      }
      incoming_state = 0;
      incoming_index = 0;
      break;
    default:
      incoming_state = 0;
      incoming_index = 0;
  }
}

int getMessageLength(uint8_t type)
{
  switch(type)
  {
    case MESSAGE_TYPE_CALIBRATION:
      return 3 + 1;
      break;
    case MESSAGE_TYPE_SET_NOZZLE:
      return 3 + 4;
      break;
    case MESSAGE_TYPE_SET_COLOR:
      return 3 + 6;
      break;
    default:
      return 0;
  }
}

void interpretMessage(uint8_t* buffer)
{
  uint8_t type = buffer[2];
  switch(type)
  {
    case MESSAGE_TYPE_SET_NOZZLE:
      interpretSetNozzleMessage(buffer); 
      break;
    case MESSAGE_TYPE_SET_COLOR:
      interpretSetColorMessage(buffer); 
      break;
    case MESSAGE_TYPE_CALIBRATION:
      interpretCalibrationMessage(buffer);
      break;
    default:
    //do nothing. 
  }
}

void interpretCalibrationMessage(uint8_t* buffer)
{
  uint8_t code = buffer[3]; //check the code. 
  if(code == CALIBRATION_COMPLETE_COMMAND)
  {
    //calibrateObjectTracker(mDetector);
    //Do something here. Resume? 
  }
}

void interpretSetColorMessage(uint8_t* buffer)
{
  //Ignore for now. 
}

void interpretSetNozzleMessage(uint8_t* buffer)
{
  uint8_t nozzle = buffer[0];
  uint32_t mode = buffer[1] << 24 | buffer[2] << 16 | buffer[3] << 8 | buffer[4]; 
  object_tracker_nozzle_modes[nozzle] = mode; 
}

void sendCalibrationMessage()
{
  static uint8_t code = START_CALIBRATION_COMMAND;
  writeToObjectDetector(MESSAGE_TYPE_CALIBRATION, &code, 1);
}



