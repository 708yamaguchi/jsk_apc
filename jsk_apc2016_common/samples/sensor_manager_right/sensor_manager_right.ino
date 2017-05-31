include <SPI.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <Wire.h>
#include <force_proximity_ros/ProximityArray.h>
#include <math.h>


unsigned long int temp_raw, pres_raw;
signed long int t_fine;

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;


#define WIRE Wire

/***** GLOBAL CONSTANTS *****/
#define NSENSORS 5
#define PCA9547D_RESET 32
#define VCNL4010_ADDRESS 0x13
#define COMMAND_0 0x80  // starts measurements, relays data ready info
#define PRODUCT_ID 0x81  // product ID/revision ID, should read 0x21
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define IR_CURRENT_VALUE  8     // range = [0, 20]. current = value * 10 mA��ή
#define AMBIENT_PARAMETER 0x84  // Configures ambient light measures
#define PROXIMITY_MOD 0x8F  // proximity modulator timing
#define LOOP_TIME 20  // loop duration in ms
#define EA 0.3  // exponential average weight parameter / cut-off frequency for high-pass filter

/***** USER PARAMETERS *****/
//int ambient_light_measurement_rate_ = 7; // range = [0, 7]. 1, 2, 3, 4, 5, 6, 8, 10 samples per second
//int averaging_function_ = 7;  // range [0, 7] measurements per run are 2**value, with range [1, 2**7 = 128]
//int proximity_freq_ = 0; // range = [0 , 3]. 390.625kHz, 781.250kHz, 1.5625MHz, 3.125MHz
//unsigned long time;

/***** GLOBAL VARIABLES *****/
unsigned int proximity_value[NSENSORS]; // current proximity reading
unsigned int average_value[NSENSORS];   // low-pass filtered proximity reading
signed int fa2[NSENSORS];              // FA-II value;
signed int fa2derivative[NSENSORS];     // Derivative of the FA-II value;
signed int fa2deriv_last[NSENSORS];     // Last value of the derivative (for zero-crossing detection)
signed int sensitivity = 1000;  // Sensitivity of touch/release detection, values closer to zero increase sensitivity

/***** ROS *****/

ros::NodeHandle nh; //write with IDE
// ros::NodeHandle_<ArduinoHardware, 1, 2, 512, 512> nh;


//unsigned long start_time;
//int continuous_mode = 1; //Default on
//int single_shot = 0;
//int touch_analysis = 1; //Default on


std_msgs::Float64 pressure_msg;
//std_msgs::Bool bool_msg;
//std_msgs::UInt16 r_finger_flex_msg;
//std_msgs::UInt16 l_finger_flex_msg;
//force_proximity_ros::Proximity proximities[NSENSORS];
force_proximity_ros::ProximityArray arr_msg;

ros::Publisher arr_pub("proximity_arr", &arr_msg);
ros::Publisher pressure_pub("gripper_front/limb/right/pressure/state", &pressure_msg);
//ros::Publisher state_pub("gripper_front/limb/right/pressure/grabbed/state", &bool_msg);
//ros::Publisher r_finger_flex_pub("gripper_front/limb/right/r_finger_flex/state", &r_finger_flex_msg);
ros::Publisher l_finger_flex_pub("gripper_front/limb/right/l_finger_flex/state", &l_finger_flex_msg);

unsigned long  publisher_timer = 0;


//////////// ADD /////////////////

int ChgI2CMultiplexer(unsigned char adrs,unsigned char ch)
{
     unsigned char c ;
     int  ans ;

     Wire.beginTransmission(adrs) ;     // �̿��γ���
     c = ch & 0x07 ;                    // ����ͥ�(bit0-2)����Ф�
     c = c | 0x08 ;                     // enable�ӥåȤ����ꤹ��
     Wire.write(c) ;                    // Control register ������
     ans = Wire.endTransmission() ;     // �ǡ������������̿��ν�λ

     return ans ;
}

byte readByte(byte address) {
  WIRE.beginTransmission(VCNL4010_ADDRESS);
  WIRE.write(address);

  debug_endTransmission(WIRE.endTransmission());
  WIRE.requestFrom(VCNL4010_ADDRESS, 1);

  while (!WIRE.available());

  byte data = WIRE.read();
  return data;
}

byte writeByte(byte address, byte data)
{
  WIRE.beginTransmission(VCNL4010_ADDRESS);
  WIRE.write(address);
  WIRE.write(data);
  return debug_endTransmission(WIRE.endTransmission());
}

unsigned int readAmbient() {
  byte temp = readByte(0x80);
  writeByte(0x80, temp | 0x10);  // command the sensor to perform ambient measure

  while (!(readByte(0x80) & 0x40)); // wait for the proximity data ready bit to be set
  unsigned int data = readByte(0x85) << 8;
  data |= readByte(0x86);

  return data;
}

unsigned int readProximity() {

  byte temp = readByte(0x80);

  writeByte(0x80, temp | 0x08);  // command the sensor to perform a proximity measure

  while (!(readByte(0x80) & 0x20)); // Wait for the proximity data ready bit to be set
  unsigned int data = readByte(0x87) << 8;
  data |= readByte(0x88);

  return data;
}

byte debug_endTransmission(int errcode)
{
  if (1)
  {
    switch (errcode)
    {
      // https://www.arduino.cc/en/Reference/WireEndTransmission
      case 0:
        //Serial.println("CAVOK");
        break;
      case 1:
        //Serial.println("data too long to fit in transmit buffer ");
        break;
      case 2:
        //Serial.println("received NACK on transmit of address ");
        break;
      case 3:
        //Serial.println("received NACK on transmit of data");
        break;
      case 4:
        //Serial.println("other error");
        break;
    }
  }
  return errcode;
}


////////// ADD /////////////



void setup() {
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(pressure_pub);
//    nh.advertise(state_pub);
//    nh.advertise(r_finger_flex_pub);
//    nh.advertise(l_finger_flex_pub);

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    initBME();
    readTrim();
}


void loop() {
    float press_act = 0.0;
    unsigned long int press_cal;

    if (millis() > publisher_timer) {
        // Pressure
        readData();
        press_cal = calibration_P(pres_raw);
        press_act = (float)press_cal / 100.0;

        //bool_msg.data = (press_act < 840);
        pressure_msg.data = press_act;

        //state_pub.publish(&bool_msg);
        pressure_pub.publish(&pressure_msg);

        // Finger Flex
        //r_finger_flex_msg.data = analogRead(A0);
        //l_finger_flex_msg.data = analogRead(A1);
        //r_finger_flex_pub.publish(&r_finger_flex_msg);
        //l_finger_flex_pub.publish(&l_finger_flex_msg);

        publisher_timer = millis() + 100;
    }
    nh.spinOnce();
}

void initBME()
{
    digitalWrite(SS, LOW);
    SPI.transfer((0xF5 & 0x7F));
    SPI.transfer(0xA0);
    SPI.transfer((0xF4 & 0x7F));
    SPI.transfer(0x27);
    digitalWrite(SS, HIGH);
}

void readTrim()
{
    uint8_t data[32];
    int i;
    digitalWrite(SS, LOW);
    SPI.transfer((0x88 | 0x80));
    for (i = 0; i < 24; i++) {
        data[i] = SPI.transfer(0);
    }
    digitalWrite(SS, HIGH);
    delay(1);
    digitalWrite(SS, LOW);
    SPI.transfer((0xA1 | 0x80));
    data[24] = SPI.transfer(0);
    digitalWrite(SS, HIGH);
    delay(1);
    digitalWrite(SS, LOW);
    SPI.transfer((0xE1 | 0x80));
    for (i = 25; i < 32; i++) {
        data[i] = SPI.transfer(0);
    }
    digitalWrite(SS, HIGH);

    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11] << 8) | data[10];
    dig_P4 = (data[13] << 8) | data[12];
    dig_P5 = (data[15] << 8) | data[14];
    dig_P6 = (data[17] << 8) | data[16];
    dig_P7 = (data[19] << 8) | data[18];
    dig_P8 = (data[21] << 8) | data[20];
    dig_P9 = (data[23] << 8) | data[22];
}

void readData()
{
    uint32_t data[8];
    int i;
    digitalWrite(SS, LOW);
    SPI.transfer((0xF7 | 0x80));
    for (i = 0; i < 8; i++) {
        data[i] = SPI.transfer(0x00);
    }
    digitalWrite(SS, HIGH);
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
}


unsigned long int calibration_P(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
    if (var1 == 0)
    {
        return 0;
    }
    P = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (P < 0x80000000)
    {
        P = (P << 1) / ((unsigned long int) var1);
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
    var2 = (((signed long int)(P >> 2)) * ((signed long int)dig_P8)) >> 13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}
