//Header Includes
#include "Adafruit_TLC5947.h"
#define BLE_serial Serial1
#define DELAY_TIME 5000

//Global Variables
uint8_t BLE_buffer[98];
uint32_t nozzle_state[7] = {1, 1, 1, 0, 1, 1, 1};
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

//Const definitions
#define DesiredPressure 1350 //Desired Value
#define PressurePin 23 //pin number
#define OpenValve 22 //pin number
#define CloseValve 21 //pin number
const int Nozzle[7] = {14, 15, 16, 17, 18, 19, 20};//pin numbers

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
//  analogReadResolution(12); //not declared in this scope
  pinMode(22, OUTPUT);  //Open
  pinMode(OpenValve,  OUTPUT);
  pinMode(CloseValve, OUTPUT);
  randomSeed(millis()%3501);
 
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
      case 1:
        nozzle_state[j] = 0;
        break;
      case 0:
        nozzle_state[j] = 1;
        break;
      case TAG_RANDOM:
        randomFunction(j);
        break;
      case TAG_CALIBRATE:

        break;
      case TAG_OB_TRACK:
        nozzle_state[j] = getObjectTrackingSuggestion(j);
        break;
      default:
        nozzle_state[j] = 1;
        break;
    }
    if(mode == 1)
    {
      nozzle_state[j] = 0;
    }
    else if(mode == 0)
    {
      nozzle_state[j] = 1;
    }
    else if(mode == tag);
    {
      randomFunction(j);
    }

    led_driver.setLED(j, red, green, blue);
    j++; //Needs to be last
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

