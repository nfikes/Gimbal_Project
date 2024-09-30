// Program used to run the Adafruit Feather ESP32-S2 TFT (Used in Arduino IDE with Libraries installed for the includes below)

// Basic demo for accelerometer & gyro readings from Adafruit
// LSM6DSO32 sensor

#include <Adafruit_LSM6DSO32.h>
#include <math.h>    //for pi and sqrt
#include <stdio.h>
//#include <armadillo>
#include <chrono>
#include <vector>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <iostream>
#include <string> // for string

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

using namespace std;
using namespace std::chrono;

// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

static float conv = M_PI/180.0;

// Initializing the initial rotational position of the gyro to be 0 in all axis.
float gyro_rot_x = 0.0;
float gyro_rot_y = 0.0;
float gyro_rot_z = 0.0;

// Initializing the system rotation to be zero, this is the target for the gimbal.
float syst_rot_x = 0.0;
float syst_rot_y = 0.0;
float syst_rot_z = 0.0;

static float motor_s1_rot = 0.0;
static float motor_s2_rot = 0.0;
static float motor_s3_rot = 0.0;

static float u_x[] = {1, 0, 0};
static float u_y[] = {0, 1, 0};
static float u_z[] = {0, 0, 1};

// Orthonormal basis vectors that each system has access to edit.
float syst_vect_x[] = {1, 0, 0};
float syst_vect_y[] = {0, 1, 0};
float syst_vect_z[] = {0, 0, 1};

float gyro_vect_x[] = {1, 0, 0};
float gyro_vect_y[] = {0, 1, 0};
float gyro_vect_z[] = {0, 0, 1};

// Weight of the error correction.
float compensation_x =  0.0001;
float compensation_y =  0.0001;
float compensation_z =  0.0001;



// PID controller Gains
float KP_gain = 1.2;
float KI_gain = 0.2;
float KD_gain = 0.0;



// Timer for Integral terms.
float s1_timer = 0.0;
float s2_timer = 0.0;
float s3_timer = 0.0;

int RX_PIN = 2;
int TX_PIN = 1;

static std::string motor_string_s1;
static std::string motor_string_s2;
static std::string motor_string_s3;

static float ang_change_sys_y = 0.0;
static float ang_change_sys_z = 0.0;

// ☆ Been Checked and confirmed works ☆ //
float vectorDot(/*vector vector*/ float v1[], /*vector vector*/ float v2[]) {
  //Vector Dot Product with lists.
  float result = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
  return result;
}

// ☆ Been Checked and confirmed works ☆ //
float clamper(float value, float u_bound, float l_bound) {
  if (value > u_bound){
    return u_bound;
  } else if (value < l_bound){
    return l_bound;
  } else {
    return value;
  }
}

// ☆ Been Checked and confirmed works ☆ //
float decider(float value, float u_bound, float l_bound) {
  if (value > u_bound){
    return 1.0;
  } else if (value < l_bound){
    return -1.0;
  } else {
    return 0.0;
  }
}

// ☆ Been Checked and confirmed works ☆ //
float* vectorScalar(float v[], float s) {
  //Scalar Multiplication
  float* result = new float[3];
  for (int i = 0; i < 3; i++) {
    result[i] = v[i] * s;
  }
  return result;
}

// ☆ Been Checked and confirmed works ☆ //
float magnitude(float v[]) {
  //Magnitude of a vector
  float a = v[0];
  float b = v[1];
  float c = v[2];
  float r = sqrt(a*a + b*b + c*c);
  return r;
}

// ☆ Been Checked and confirmed works ☆ //
float* vectorProject(/*vector vector*/ float proj_v[], /*vector vector*/ float onto_v[]) {
  // Perform vector projection:
  
  float norm = magnitude(onto_v);
  float dot = vectorDot(onto_v, proj_v);
  float vec_size = (dot / (norm * norm));
  float* projected_vector = vectorScalar(onto_v, vec_size);
  
  return projected_vector;
}

// ☆ Been Checked and confirmed works ☆ //
float* fixedRotation(/*vector vector*/ float vector[], float a, float b, float g) {
  // Pre multiply a vector in 3D space with the fundamental rotation matrix.
  // a = yaw = z and b = pitch = y

  float* result = new float[3];

  a = a*conv;
  b = b*conv;
  g = g*conv;

  float row1[] = {cos(a)*cos(b),   cos(a)*sin(b)*sin(g) - sin(a)*cos(g),   cos(a)*sin(b)*cos(g) + sin(a)*sin(g)};
  float row2[] = {sin(a)*cos(b),   sin(a)*sin(b)*sin(g) + cos(a)*cos(g),   sin(a)*sin(b)*cos(g) - cos(a)*sin(g)};
  float row3[] = {      -sin(b),                          cos(b)*sin(g),                          cos(b)*cos(g)};

  result[0] = vectorDot(row1, vector);
  result[1] = vectorDot(row2, vector);
  result[2] = vectorDot(row3, vector);

  return result;
}

// ☆ Been Checked and confirmed works ☆ //
float* relativeRotation(/*vector vector*/ float vector[], float t, float axis[]) {
  // Pre multiply a vector in 3D space with the axial rotation matrix.
  float ux = axis[0];
  float uy = axis[1];
  float uz = axis[2];

  float* result = new float[3];

  t = t*conv;

  float row1[] = {  cos(t) + (ux*ux)*(1-cos(t)),   ux*uy*(1-cos(t)) - uz*sin(t),   ux*uz*(1-cos(t)) + uy*sin(t)};
  float row2[] = {uy*ux*(1-cos(t)) + uz*sin(t),     cos(t) + (uy*uy)*(1-cos(t)),   uy*uz*(1-cos(t)) - ux*sin(t)};
  float row3[] = {uz*ux*(1-cos(t)) - uy*sin(t),   uz*uy*(1-cos(t)) + ux*sin(t),     cos(t) + (uz*uz)*(1-cos(t))};

  result[0] = vectorDot(row1, vector);
  result[1] = vectorDot(row2, vector);
  result[2] = vectorDot(row3, vector);

  return result;
}

// ☆ Been Checked and confirmed works ☆ //
void oledDrawText(String text, uint16_t color) {
  static int x = 5;
  static int y = 5;

  if (text == "\n") {
    y = y + 10;
  }

  tft.setCursor(x, y);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

Adafruit_LSM6DSO32 dso32;
void setup(void) {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  delay(500);
  //while (!Serial)
  //delay(10); // will pause Zero, Leonardo, etc until Serial console opens

  Serial.println("Adafruit LSM6DSO32 test!");

  if (!dso32.begin_I2C()) {
    // if (!dso32.begin_SPI(LSM_CS)) {
    // if (!dso32.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DSO32 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DSO32 Found!");

  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (dso32.getAccelRange()) {
  case LSM6DSO32_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DSO32_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DSO32_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  case LSM6DSO32_ACCEL_RANGE_32_G:
    Serial.println("+-32G");
    break;
  }

  // dso32.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  switch (dso32.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSO32
  }

  // dso32.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (dso32.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (dso32.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  Serial.println(F("Initialized"));

  // Used for the compensation factors.
  pinMode(18, INPUT);
  pinMode(17, INPUT);
  pinMode(16, INPUT);

  //Button
  pinMode(5, INPUT);

  // Joystick
  pinMode(8, INPUT);
  pinMode(14, INPUT);
  pinMode(13, INPUT);

  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(500);

  tft.init(135, 240); // Init ST7789 240x135
  tft.setRotation(3);
  tft.setTextColor(ST77XX_WHITE);
  tft.fillScreen(ST77XX_BLACK);

  oledDrawText("--------------------------------------", ST77XX_BLUE);
  oledDrawText("\n", ST77XX_WHITE);
  oledDrawText("Gyroscopic Sensor disabled, device    ", ST77XX_WHITE);
  oledDrawText("                                   OFF", ST77XX_RED);
  oledDrawText("\n", ST77XX_WHITE);
  oledDrawText("The                                   ", ST77XX_WHITE);
  oledDrawText("     RED                              ", ST77XX_RED);
  oledDrawText("         button will reset the gyro   ", ST77XX_WHITE);
  oledDrawText("\n", ST77XX_WHITE);
  oledDrawText("angular position to                   ", ST77XX_WHITE);
  oledDrawText("                    [0,0,0]           ", ST77XX_YELLOW);
  oledDrawText("                            A Yellow  ", ST77XX_WHITE);
  oledDrawText("\n", ST77XX_WHITE);
  oledDrawText("light will shine when the             ", ST77XX_WHITE);
  oledDrawText("                          RED         ", ST77XX_RED);
  oledDrawText("                              button  ", ST77XX_WHITE);
  oledDrawText("\n", ST77XX_WHITE);
  oledDrawText("is pressed to show successful zeroing.", ST77XX_WHITE);
  oledDrawText("\n", ST77XX_WHITE);
  oledDrawText("--------------------------------------", ST77XX_BLUE);
  oledDrawText("\n", ST77XX_WHITE);
  oledDrawText("\n", ST77XX_WHITE);
  oledDrawText("   ANTEIKA 5 Portable Gimbal Device   ", ST77XX_GREEN);
  oledDrawText(" [                                  ] ", ST77XX_WHITE);
  oledDrawText("\n", ST77XX_WHITE);
  oledDrawText("\n", ST77XX_WHITE);
  oledDrawText("--------------------------------------", ST77XX_BLUE);
  oledDrawText("\n", ST77XX_WHITE);
  oledDrawText("Please use device slowly with care.   ", ST77XX_MAGENTA);
  oledDrawText("\n", ST77XX_WHITE);
  oledDrawText("Calbration POTS are [X] [Y] [Z] order.", ST77XX_WHITE);

  Serial.println(F("Screen Printed"));
}

void loop() {
  /*
  auto start = high_resolution_clock::now();
  */
  // Sets a frequency at which to sample data.
  delay(100);

  auto reset = digitalRead(5);
  if (reset > 0) {
    gyro_rot_x = 0.0;
    gyro_rot_y = 0.0;
    gyro_rot_z = 0.0;
  }
  /*
  s1_timer += 1.0;
  s2_timer += 1.0;
  s3_timer += 1.0;
  */

  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  dso32.getEvent(&accel, &gyro, &temp);
  /*
  float driver_x = (analogRead(18) / 100.0) - 47.0;
  float driver_y = (analogRead(17) / 100.0) - 47.0;
  float driver_z = (analogRead(16) / 100.0) - 47.0;
  */
  
  // -------------------------------------------------- //
  // Read the (x and y) potentiometers of the joystick as voltages and map them to (y and z) of the system basis. (Roll will not be used.)
  float joy_z = analogRead(8);
  float joy_y = analogRead(14);
  float reset_axis = analogRead(13);

  if (decider(reset_axis, 2000.0, 100.0) == 1.0) {
    ang_change_sys_y = 0.0;
    ang_change_sys_z = 0.0;
    motor_s1_rot = 0.0;
    motor_s2_rot = 0.0;
    motor_s3_rot = 0.0;
  }
  
  ang_change_sys_y += 10.0 * decider(joy_y, 7000.0, 6000.0); //analogRead(pin) [Pitch]
  ang_change_sys_z += 10.0 * decider(joy_z, 7000.0, 6000.0); //analogRead(pin) [Yaw]

  /*
  float* syst_vect_x = fixedRotation(u_x, ang_change_sys_z, ang_change_sys_y, 0.0);
  float* syst_vect_y = fixedRotation(u_y, ang_change_sys_z, ang_change_sys_y, 0.0);
  float* syst_vect_z = fixedRotation(u_z, ang_change_sys_z, ang_change_sys_y, 0.0);

  float* gyro_vect_x = fixedRotation(u_x, gyro_rot_x * 1/conv, gyro_rot_y * 1/conv, gyro_rot_z * 1/conv);
  float* gyro_vect_y = fixedRotation(u_y, gyro_rot_x * 1/conv, gyro_rot_y * 1/conv, gyro_rot_z * 1/conv);
  float* gyro_vect_z = fixedRotation(u_z, gyro_rot_x * 1/conv, gyro_rot_y * 1/conv, gyro_rot_z * 1/conv);

  // -------------------------------------------------- //
  // Dead recon of the vector directions based on motor rotations:
  float* motor_vect_s1 = gyro_vect_x;

  float* motor_vect_s2 = relativeRotation(gyro_vect_y, motor_s1_rot, motor_vect_s1);

  float* intermediate_ = relativeRotation(gyro_vect_z, motor_s1_rot, motor_vect_s1);

  float* motor_vect_s3 = relativeRotation(intermediate_, motor_s2_rot, motor_vect_s2);

  // -------------------------------------------------- //
  // Motor influences:
  float s1_x_inf = vectorDot(syst_vect_x, vectorProject(motor_vect_s1, syst_vect_x));
  float s1_y_inf = vectorDot(syst_vect_y, vectorProject(motor_vect_s1, syst_vect_y));
  float s1_z_inf = vectorDot(syst_vect_z, vectorProject(motor_vect_s1, syst_vect_z));

  float s2_x_inf = vectorDot(syst_vect_x, vectorProject(motor_vect_s2, syst_vect_x));
  float s2_y_inf = vectorDot(syst_vect_y, vectorProject(motor_vect_s2, syst_vect_y));
  float s2_z_inf = vectorDot(syst_vect_z, vectorProject(motor_vect_s2, syst_vect_z));

  float s3_x_inf = vectorDot(syst_vect_x, vectorProject(motor_vect_s3, syst_vect_x));
  float s3_y_inf = vectorDot(syst_vect_y, vectorProject(motor_vect_s3, syst_vect_y));
  float s3_z_inf = vectorDot(syst_vect_z, vectorProject(motor_vect_s3, syst_vect_z));

  // -------------------------------------------------- //
  // Negated Error between system and gyro: (Stable equilibrium) x-yz y-zx z-xy
  float e_x = - vectorDot(syst_vect_z, vectorProject(gyro_vect_y, syst_vect_z));
  float e_y = - vectorDot(syst_vect_x, vectorProject(gyro_vect_z, syst_vect_x));
  float e_z = - vectorDot(syst_vect_y, vectorProject(gyro_vect_x, syst_vect_y));

  // -------------------------------------------------- //
  // Motor rotation target
  float e_s1 = e_x * s1_x_inf + e_y * s1_y_inf + e_z * s1_z_inf;
  float e_s2 = e_x * s2_x_inf + e_y * s2_y_inf + e_z * s2_z_inf;
  float e_s3 = e_x * s3_x_inf + e_y * s3_y_inf + e_z * s3_z_inf;

  // -------------------------------------------------- //
  float o_e_s1 = e_s1;
  float o_e_s2 = e_s2;
  float o_e_s3 = e_s3;
  
  auto stop = high_resolution_clock::now();

  // However long it takes the loop to happen minus the gyro integration calculation time, serves as dt for the step
  auto timer = duration_cast<milliseconds>(stop - start);

  if (abs(e_s1) < 0.01){
    s1_timer = 0.0;
  }
  if (abs(e_s2) < 0.01){
    s2_timer = 0.0;
  }
  if (abs(e_s3) < 0.01){
    s3_timer = 0.0;
  }

  
  // PID system on rotation target parameter. Control Function.
  // Control Function (Pseudo Integral term)
  float u_s1 = KP_gain * e_s1 + KI_gain * e_s1 * s1_timer + KD_gain * ((e_s1 - o_e_s1) / float(timer.count()));
  float u_s2 = KP_gain * e_s2 + KI_gain * e_s2 * s2_timer + KD_gain * ((e_s2 - o_e_s2) / float(timer.count()));
  float u_s3 = KP_gain * e_s3 + KI_gain * e_s3 * s3_timer + KD_gain * ((e_s3 - o_e_s3) / float(timer.count()));
  
  //Applying Control Function to Motor Rotation
  motor_s1_rot += u_s1;
  motor_s2_rot += u_s2;
  motor_s3_rot += u_s3;
  */

  //Temporary fix and lowering the scope of the project.
  motor_s1_rot = 0.0;
  motor_s2_rot = ang_change_sys_y;
  motor_s3_rot = ang_change_sys_z;

  
  // Clamping so the values don't run away.
  motor_s1_rot = clamper(motor_s1_rot, 90.0, -90.0);
  motor_s2_rot = clamper(motor_s2_rot, 90.0, -90.0);
  motor_s3_rot = clamper(motor_s3_rot, 90.0, -90.0);

  //Makes sure that the stack of stuff on the line isnt being overloaded.

  if (Serial1.availableForWrite() > 10) {
    motor_string_s1 += 'A';
    motor_string_s1 += std::to_string( motor_s1_rot );
    motor_string_s1 += '\n';

    motor_string_s2 += 'B';
    motor_string_s2 += std::to_string( motor_s2_rot );
    motor_string_s2 += '\n';

    motor_string_s3 += 'C';
    motor_string_s3 += std::to_string( motor_s3_rot );
    motor_string_s3 += '\n';

    Serial1.write(motor_string_s1.c_str());
    Serial1.write(motor_string_s2.c_str());
    Serial1.write(motor_string_s3.c_str());
    motor_string_s1.clear();
    motor_string_s2.clear();
    motor_string_s3.clear();

  }

  // Numerical Integration of Gyro Axis DPS measurements with linear Error Correction (I know it is bad but it is the best short term fix.)
  /*
  gyro_rot_x = gyro_rot_x + gyro.gyro.x * float(timer.count())/1000.0 + compensation_x * driver_x;
  gyro_rot_y = gyro_rot_y + gyro.gyro.y * float(timer.count())/1000.0 + compensation_y * driver_y;
  gyro_rot_z = gyro_rot_z + gyro.gyro.z * float(timer.count())/1000.0 + compensation_z * driver_z;
  */

  // Gyro has an onboard Thermometer and Acceleration device which will not be used.

  /* Display the results (rotation is measured in rad/s) */
  /*
  Serial.print(ang_change_sys_y);
  Serial.println(" : system y rot\n");
  Serial.print(ang_change_sys_z);
  Serial.println(" : system z rot\n");
  Serial.println("\n");
  Serial.print(gyro_rot_x * 1/conv);
  Serial.println(" : x gyro\n");
  Serial.print(gyro_rot_y * 1/conv);
  Serial.println(" : y gyro\n");
  Serial.print(gyro_rot_z * 1/conv);
  Serial.println(" : z gyro\n\n");
  */
}