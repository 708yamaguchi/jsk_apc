#include <SPI.h>
#include <ros.h>
#include <Wire.h>
#include <jsk_arc2017_baxter/GripperSensorStates.h>

#define WIRE Wire

/***** GLOBAL CONSTANTS *****/

#define VCNL4040_ADDR 0x60 //7-bit unshifted I2C address of VCNL4040

//Command Registers have an upper byte and lower byte.
#define ALS_CONF 0x00
#define PS_CONF1 0x03
//#define PS_CONF2 //High byte of PS_CONF1
#define PS_CONF3 0x04
//#define PS_MS //High byte of PS_CONF3
#define PS_DATA_L 0x08
//#define PS_DATA_M //High byte of PS_DATA_L
#define ALS_DATA_L 0x09
//#define ALS_DATA_M //High byte of ALS_DATA_L
#define WHITE_DATA_L 0x0A
//#define WHITE_DATA_M //High byte of WHITE_DATA_L
#define ID  0x0C


#define NSENSORS 1
#define PCA9547D_RESET 32
#define VCNL4010_ADDRESS 0x60
#define COMMAND_0 0x80  // starts measurements, relays data ready info
#define PRODUCT_ID 0x81  // product ID/revision ID, should read 0x21
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define IR_CURRENT_VALUE 8  // range = [0, 20]. current = value * 10 mA��ή
#define AMBIENT_PARAMETER 0x84  // Configures ambient light measures
#define PROXIMITY_MOD 0x8F  // proximity modulator timing
#define LOOP_TIME 50  // loop duration in ms
#define EA 0.3  // exponential average weight parameter / cut-off frequency for high-pass filter
#define sensitivity 1000  // Sensitivity of touch/release detection, values closer to zero increase sensitivity

/***** GLOBAL VARIABLES *****/
unsigned int average_value[NSENSORS];   // low-pass filtered proximity reading
signed int fa2[NSENSORS];              // FA-II value;

int ChgI2CMultiplexer(unsigned char adrs,unsigned char ch)
{
  unsigned char c;
  int  ans;

  Wire.beginTransmission(adrs);
  c = ch & 0x07;
  c = c | 0x08;
  Wire.write(c);
  ans = Wire.endTransmission();

  return ans ;
}

void measure_proximity()
{
  int i;
  for(i=0;i<NSENSORS;i++)
    {
      ChgI2CMultiplexer(0x70, i);

      delay(1);
      unsigned int proximity_value = readFromCommandRegister(PS_DATA_L);
      unsigned int ambient_value = readFromCommandRegister(ALS_DATA_L);
      unsigned int white_value = readFromCommandRegister(WHITE_DATA_L);

      signed int fa2derivative = (signed int) average_value[i] - proximity_value - fa2[i];
      fa2[i] = (signed int) average_value[i] - proximity_value;
      /*
      proximities[i].proximity = proximity_value;
      proximities[i].average = average_value[i];
      proximities[i].fa2 = fa2[i];
      proximities[i].fa2derivative = fa2derivative;
      if (fa2[i] < -sensitivity) proximities[i].mode = "T";
      else if (fa2[i] > sensitivity) proximities[i].mode = "R";
      else proximities[i].mode = "0";
      */
      average_value[i] = EA * proximity_value + (1 - EA) * average_value[i];

      Serial.print("proximity value : ");
      Serial.print(proximity_value);
      Serial.print("  ambient_value");
      Serial.print(ambient_value);
      Serial.print("  white_value");
      Serial.println(white_value);

      delay(1);

    }
}

void initVCNL4040()
{
  //Clear PS_SD to turn on proximity sensing
  //byte conf1 = 0b00000000; //Clear PS_SD bit to begin reading
  byte conf1 = 0b00001110; //Integrate 8T, Clear PS_SD bit to begin reading
  byte conf2 = 0b00001000; //Set PS to 16-bit
  //byte conf2 = 0b00000000; //Clear PS to 12-bit
  writeToCommandRegister(PS_CONF1, conf1, conf2); //Command register, low byte, high byte

  //Set the options for PS_CONF3 and PS_MS bytes
  byte conf3 = 0x00;
  //byte ms = 0b00000010; //Set IR LED current to 100mA
  //byte ms = 0b00000110; //Set IR LED current to 180mA
  byte ms = 0b00000111; //Set IR LED current to 200mA
  writeToCommandRegister(PS_CONF3, conf3, ms);

  //Set the options for ALS_CONF
  byte conf = 0b00000000; //Clear the 
  writeToCommandRegister(ALS_CONF, conf, 0x00);
}

//Reads a two byte value from a command register
unsigned int readFromCommandRegister(byte commandCode)
{
  delay(1);
  Wire.beginTransmission(VCNL4040_ADDR);
  delay(1);
  Wire.write(commandCode);
  delay(1);
  Wire.endTransmission(false); //Send a restart command. Do not release bus.

  delay(1);
  Wire.requestFrom(VCNL4040_ADDR, 2); //Command codes have two bytes stored in them

  delay(1);
  unsigned int data = Wire.read();
  data |= Wire.read() << 8;

  return (data);
}


//Write a two byte value to a Command Register
void writeToCommandRegister(byte commandCode, byte lowVal, byte highVal)
{
  Wire.beginTransmission(VCNL4040_ADDR);
  Wire.write(commandCode);
  Wire.write(lowVal); //Low byte of command
  Wire.write(highVal); //High byte of command
  Wire.endTransmission(); //Release bus
}


void setup()
{

  Serial.begin(9600);
  
  pinMode(PCA9547D_RESET, OUTPUT);
  digitalWrite(PCA9547D_RESET, HIGH);

  Wire.begin(); //Join i2c bus

  Serial.println("setup");

  // this is essential code. I cannnot understand why this code is needed.
  /* MYSTERY */
  int deviceID = readFromCommandRegister(ID);
  if (deviceID != 0x186)
    {
      while (1); //Freeze!
    }
  /* MYSTERY */

  initVCNL4040(); //Configure sensor

  int i;
  for(i=0;i<NSENSORS;i++)
    {
      ChgI2CMultiplexer(0x70,i);
      average_value[i] = readFromCommandRegister(PS_DATA_L);;
      fa2[i] = 0;
    }

}


void loop()
{
  unsigned long time;
  time = millis();

  delay(1);

  measure_proximity();

  while (millis() < time + LOOP_TIME); // enforce constant loop time

}
