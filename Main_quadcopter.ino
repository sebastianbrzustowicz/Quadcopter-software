/**
 * This is main file devoted for on board usage.
 *
 * @author Sebastian Brzustowicz <Se.Brzustowicz@gmail.com>
 */
// ---------------------------------------------------------------------------
#define data_sample_size 20000
// ---------------------------------------------------------------------------
#include "Wire.h"
#include <MPU6050_light.h>
MPU6050 mpu(Wire1);
// ---------------------------------------------------------------------------
#include <BMP388_DEV.h>                           // Include the BMP388_DEV.h library
float temperature, pressure;                      // Create the temperature, pressure and *altitude* variables
BMP388_DEV bmp388;                                // Instantiate (create) a BMP388_DEV object and set-up for I2C operation (address 0x77)
// ---------------------------------------------------------------------------
#include <Servo.h>
// ---------------------------------------------------------------------------
// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
// ---------------------------------------------------------------------------
Servo motA, motB, motC, motD;
char data;
// ---------------------------------------------------------------------------
//Receive data
float received_data[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int testing_val = -1;
int time_no_signal = -20000;    // not starting immidiately, just on command
//GSR
float altitude_d = 0;
float roll_d = 0;
float pitch_d = 0;
float yaw_d = 0;
//Controller parameters
bool controller_tune = 0;
float altitude_kp = 0;
float altitude_ki = 0;
float altitude_kd = 0;
float roll_kp     = 5;  //8
float roll_ki     = 0;
float roll_kd     = 800; //800
float pitch_kp    = 5;
float pitch_ki    = 0;
float pitch_kd    = 800;
float yaw_kp      = 0;
float yaw_ki      = 0;
float yaw_kd      = 0;
//Barometer
float altitude = 0;
//IMU
float roll     = 0;
float pitch    = 0;
float yaw      = 0;
//Controller
float sample_time         = 3; // ms
unsigned long current_time = 0;
unsigned long last_time    = 0;
int delta_time             = 0;
float e_altitude =       0;
float e_altitude_total = 0;
float e_altitude_delta = 0;
float e_altitude_last =  0;
float e_roll =           0;
float e_roll_total =     0;
float e_roll_delta =     0;
float e_roll_last =      0;
float e_pitch =          0;
float e_pitch_total =    0;
float e_pitch_delta =    0;
float e_pitch_last =     0;
float e_yaw =            0;
float e_yaw_total =      0;
float e_yaw_delta =      0;
float e_yaw_last =       0;

float Kc = 0.02;

int max_control = 3;
int min_control = -3;

float U1 = 0;
float U2 = 0;
float U3 = 0;
float U4 = 0;
float U1_sat = 0;
float U2_sat = 0;
float U3_sat = 0;
float U4_sat = 0;
float F1 = 0;
float F2 = 0;
float F3 = 0;
float F4 = 0;
float F1_sat = 0;
float F2_sat = 0;
float F3_sat = 0;
float F4_sat = 0;
//Signal conversion

//Motors set
float pwm1 = 1000;
float pwm2 = 1000;
float pwm3 = 1000;
float pwm4 = 1000;
//Data storage
float sample_array[data_sample_size]= {0};
int iterator = 0;
void setup() {
    //delete serial later
    Serial.begin(115200);
    Serial1.begin(9600);    //check 115200 speed of bluetooth in terms of basic task
    //IMU
    Wire1.begin();
    byte status = mpu.begin();
    //Serial.println(status);
    mpu.calcOffsets();
    //Serial.println("Done!\n");
    //Barometer
    bmp388.begin();
    //Motors
    motA.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motB.attach(5, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motC.attach(7, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motD.attach(4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

    motA.writeMicroseconds(MIN_PULSE_LENGTH);
    motB.writeMicroseconds(MIN_PULSE_LENGTH);
    motC.writeMicroseconds(MIN_PULSE_LENGTH);
    motD.writeMicroseconds(MIN_PULSE_LENGTH);
}

void loop() {
//while(millis()<5000){Serial.println("wait");}
Receive_data();

GSR_controller_params();
//Serial.println(e_pitch_total);
Read_IMU();

//Read_Barometer();
//Serial.println(pitch);
//Data_storage();
//Send_Data();
//Serial.print(pitch);
//Serial.print("    ");
//Serial.print(e_pitch);
//Serial.print("    ");
//Serial.println(e_pitch_delta);

Controller();
//Serial.print(pwm1);
//Serial.print("    ");
//Serial.print(pwm2);
//Serial.print("    ");
//Serial.print(pwm3);
//Serial.print("    ");
//Serial.println(pwm4);

Motors_set();

//Serial.println(e_pitch_delta);
//iterator = iterator + 1;
//while(millis()>15000){
//  Serial.println(iterator/10);
//  }
}

// Receiving desired signals through bluetooth module
void Receive_data()
{
if (Serial1.available()>=17) {
  testing_val = Serial1.read();
  if (testing_val !=- 1){
    received_data[0] = testing_val;
  for ( int i = 1; i < 17; i++ ) {
  do{
  received_data[i] = Serial1.read();
  }while(received_data[i] == -1);
  }
  }
  }
}

void GSR_controller_params(){
  if (testing_val !=- 1){
  testing_val = -1;
  altitude_d = received_data[0];
  roll_d =     received_data[1];
  pitch_d =    received_data[2];
  yaw_d =      received_data[3];
  e_roll_total = 0;
  e_pitch_total = 0;
  e_yaw_total = 0;
  e_altitude_total = 0;
  controller_tune = received_data[16];
  if(controller_tune == 1){
  altitude_kp =     received_data[4];
  altitude_ki =     received_data[5];
  altitude_kd =     received_data[6];
  roll_kp =    received_data[7];
  roll_ki =    received_data[8];
  roll_kd =    received_data[9];
  pitch_kp =   received_data[10];
  pitch_ki =   received_data[11];
  pitch_kd =   received_data[12];
  yaw_kp =     received_data[13];
  yaw_ki =     received_data[14];
  yaw_kd =     received_data[15];
  controller_tune = 0;}
  time_no_signal = millis();
  /*
  Serial.println("Actual data");
  Serial.println(altitude_d);
  Serial.println(roll_d);
  Serial.println(pitch_d);
  Serial.println(yaw_d);
  Serial.println(altitude_kp);
  Serial.println(altitude_ki);
  Serial.println(altitude_kd);
  Serial.println(roll_kp);
  Serial.println(roll_ki);
  Serial.println(roll_kd);
  Serial.println(pitch_kp);
  Serial.println(pitch_ki);
  Serial.println(pitch_kd);
  Serial.println(yaw_kp);
  Serial.println(yaw_ki);
  Serial.println(yaw_kd);
  Serial.println(controller_tune);
  */
  }

  // ramp/SVF_filter eventually

}

void Read_IMU(){
mpu.update();
roll =  mpu.getAngleX();
pitch = mpu.getAngleY();
yaw =   mpu.getAngleZ();

//Serial.println("IMU");
//Serial.println(roll);
//Serial.print("  ");
//Serial.println(pitch);
//Serial.print("  ");
//Serial.println(yaw);

}

void Read_Barometer(){
  bmp388.startForcedConversion();                 // Start a forced conversion (if in SLEEP_MODE)
  if (bmp388.getMeasurements(temperature, pressure, altitude))    // Check if the measurement is complete
  {
    //altitude = altitude*100;
    //Serial.print(temperature);                    // Display the results    
    //Serial.print(F("*C   "));
    //Serial.print(pressure);    
    //Serial.print(F("hPa   "));
    //Serial.print(F("Alt"));
    //Serial.println(altitude);
    //Serial.println(F("m"));
    
  }
}

void Data_storage(){
sample_array[iterator] = roll;
iterator = iterator + 1;
if (iterator>=data_sample_size)
{
  for( int i = 0; i <= data_sample_size; i++ ) {
  Serial1.println(sample_array[i]);}
  iterator = 0;
}
}

void Send_Data(){
//Serial1.write(roll);
//Serial1.write(pitch);
//Serial1.write(yaw);
//Serial1.write(altitude);

//Serial.println(roll);
//Serial.println(pitch);
//Serial.println(yaw);
//Serial.println(altitude*100);

//Serial.println(roll);
//Serial.println(roll_d);
Serial.println(pitch);
Serial.println(pitch_d);
}

void Controller(){
// kp errors
e_altitude = altitude_d -altitude;
e_roll =     roll_d - roll;
e_pitch =    pitch_d - pitch;
e_yaw =      yaw_d - yaw;
// ki, kd errors
e_altitude_total += e_altitude;
e_altitude_delta  = e_altitude - e_altitude_last;
e_altitude_last   = e_altitude;

e_roll_total += e_roll; // error for ki
//if (e_roll_total >= max_control) e_roll_total = max_control;
//else if (e_roll_total <= min_control) e_roll_total = min_control;
e_roll_delta = e_roll - e_roll_last; // error for kd
e_roll_last  = e_roll;
Kc = 0.02;
e_pitch_total += e_pitch;// + Kc*(U3_sat - U3);
if (e_pitch_total >= max_control) e_pitch_total = max_control;
else if (e_pitch_total <= min_control) e_pitch_total = min_control;
e_pitch_delta  = e_pitch - e_pitch_last;
e_pitch_last   = e_pitch;

e_yaw_total += e_yaw;
e_yaw_delta  = e_yaw - e_yaw_last;
e_yaw_last   = e_yaw;

//U2 = roll_kp*e_roll + (roll_ki*sample_time)*e_roll_total + (roll_kd/sample_time)*e_roll_delta; //PID control signal compute
//if (U2 >= max_control) U2 = max_control;
//else if (U2 <= min_control) U2 = min_control;

last_time   = current_time;

// //PID control signals computing
U1 = altitude_kp*e_altitude + (altitude_ki*sample_time)*e_altitude_total + (altitude_kd/sample_time)*e_altitude_delta;
U2 = roll_kp*e_roll         + (roll_ki*sample_time)*e_roll_total         + (roll_kd/sample_time)*e_roll_delta;
U3 = pitch_kp*e_pitch       + (pitch_ki*sample_time)*e_pitch_total       + (pitch_kd/sample_time)*e_pitch_delta;
U4 = yaw_kp*e_yaw           + (yaw_ki*sample_time)*e_yaw_total           + (yaw_kd/sample_time)*e_yaw_delta;

//Serial.print(U3);

//F1_sat = min(900, max(0, F1));
//F2_sat = min(900, max(0, F2));
//F3_sat = min(900, max(0, F3));
//F4_sat = min(900, max(0, F4));

//U1_sat = F1_sat + F2_sat + F3_sat + F4_sat; // 0 - 36
//U2_sat = F1_sat + F2_sat - F3_sat - F4_sat; // -18 - 18
//U3_sat = -F1_sat + F2_sat - F3_sat + F4_sat;// -18 - 18
//U4_sat = -F1_sat + F2_sat + F3_sat - F4_sat;// -18 - 18

U1_sat = min(3000, max(0, U1)); //0 - 360
U2_sat = min(1200, max(-1200, U2)); //-180 - 180
U3_sat = min(1200, max(-1200, U3)); //-180 - 180
U4_sat = min(1200, max(-1200, U4)); //-180 - 180

F1 = U1_sat + U2_sat - U3_sat - U4_sat;
F2 = U1_sat + U2_sat + U3_sat + U4_sat;
F3 = U1_sat - U2_sat - U3_sat + U4_sat;
F4 = U1_sat - U2_sat + U3_sat - U4_sat;

//Serial.print(F1);
//Serial.print("   ");
//Serial.print(F2);
//Serial.print("   ");
//Serial.print(F3);
//Serial.print("   ");
//Serial.println(F4);

// Force to PWM (0, 10) ---> (1000, 2000)
if (F1 >= 0) pwm1 = sqrt(F1*0.0022 + 0.2282)*929.4066+555.9760; else pwm1 = 1000;
if (F2 >= 0) pwm2 = sqrt(F2*0.0022 + 0.2282)*929.4066+555.9760; else pwm2 = 1000;
if (F3 >= 0) pwm3 = sqrt(F3*0.0022 + 0.2282)*929.4066+555.9760; else pwm3 = 1000;
if (F4 >= 0) pwm4 = sqrt(F4*0.0022 + 0.2282)*929.4066+555.9760; else pwm4 = 1000;

pwm1 = pwm1 + 200;
pwm2 = pwm2 + 200;
pwm3 = pwm3 + 200;
pwm4 = pwm4 + 200;

// PWM saturation <1000, 2000>
pwm1 = min(MAX_PULSE_LENGTH, max(MIN_PULSE_LENGTH, pwm1));
pwm2 = min(MAX_PULSE_LENGTH, max(MIN_PULSE_LENGTH, pwm2));
pwm3 = min(MAX_PULSE_LENGTH, max(MIN_PULSE_LENGTH, pwm3));
pwm4 = min(MAX_PULSE_LENGTH, max(MIN_PULSE_LENGTH, pwm4));

//Serial.print(pwm1);
//Serial.print("    ");
//Serial.print(pwm2);
//Serial.print("    ");
//Serial.print(pwm3);
//Serial.print("    ");
//Serial.println(pwm4);
do{
current_time = millis();
delta_time = current_time - last_time;}
while (delta_time < sample_time);
}

// Function for sending appropriate pwm signals to motors
void Motors_set()
{
  if (millis()-time_no_signal<=1500){
  // delete serial prints and delay later
  //Serial.print("Pulse length = ");
  //Serial.print(pwm1);
  //Serial.print("   ");
  //Serial.print(pwm2);
  //Serial.print("   ");
  //Serial.print(pwm3);
  //Serial.print("   ");
  //Serial.println(pwm4);
  //
  motA.writeMicroseconds(pwm1);
  motB.writeMicroseconds(pwm2);
  motC.writeMicroseconds(pwm3);
  motD.writeMicroseconds(pwm4);
  
  }else
  {
  motA.writeMicroseconds(MIN_PULSE_LENGTH);
  motB.writeMicroseconds(MIN_PULSE_LENGTH);
  motC.writeMicroseconds(MIN_PULSE_LENGTH);
  motD.writeMicroseconds(MIN_PULSE_LENGTH);
  //Serial.println("Signals = 1000");
  }
  //delay(200);
  //sampling
}
