#include <PinChangeInt.h>
#include <MsTimer2.h>
// Realization of Speed ​​PID Control by Encoder Counting
#include <BalanceCar.h>
#include <KalmanFilter.h>
// The I2Cdev, MPU6050 class libraries need to be installed in the Arduino class library folder in advance
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu; // Instantiate an MPU6050 object with the object name mpu
BalanceCar balancecar;
KalmanFilter kalmanfilter;
int16_t ax, ay, az, gx, gy, gz;
// TB6612FNG Drive module control signal
#define IN1M 7
#define IN2M 6
#define IN3M 13
#define IN4M 12
#define PWMA 9
#define PWMB 10
#define STBY 8
// Encoder count signal
#define PinA_left 2  // interrupt 0
#define PinA_right 4 // interrupt 1
// declare custom variable
int time;
byte inByte; // serial port receive bytes
int num;
double Setpoint; // Angle DIP set point, input, output
double Setpoints, Outputs = 0; // Speed ​​DIP set point, input, output
double kp = 40, ki = 0.0, kd = 0.6; // Parameters that need you to modify, angle PD control
double kp_speed =5.20, ki_speed = 0.25, kd_speed = 0.0; // Parameters you need to modify, speed PI control
double kp_turn = 23, ki_turn = 0, kd_turn = 0.3;// Rotate PD settings
const double PID_Original[6] = {40, 0.0, 0.6, 5.20, 0.25, 0.0}; // Restore default PID parameters
// Steering PID parameters
double setp0 = 0, dpwm = 0, dl = 0; // Angle balance point, PWM difference, dead time, PWM1, PWM2
float value;
/******************** angle data *********************/
float Q;
float Angle_ax; // Inclination angle calculated from acceleration
float Angle_ay;
float K1 = 0.05; // Weights for accelerometer values
float angle0 = 0.00; // Mechanical balance angle
int slong;
/***************Kalman_Filter*********************/
float Q_angle = 0.001, Q_gyro = 0.005; // Confidence of angle data, Confidence of angular velocity data
float R_angle = 0.5 , C_0 = 1;
float timeChange = 5; // Filter method sampling interval milliseconds
float dt = timeChange * 0.001; // Note: The value of dt is the filter sampling time
/***************Kalman_Filter*********************/
/******************** speed count ************/
volatile long count_right = 0; // The volatile lon type is used to ensure that the value is valid when the external interrupt pulse count value is used in other functions
volatile long count_left = 0; // The volatile lon type is used to ensure that the value is valid when the external interrupt pulse count value is used in other functions
int speedcc = 0;
/******************** Pulse calculation ********************/
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int sumam;
/******************** Steering and rotation parameters ********************/
int turncount = 0; // Steering to Intervention Time Calculation
float turnoutput = 0;
/******************** Bluetooth control amount ********************/
#define run_car     '1' // before the key
#define back_car    '2' // after keypress
#define left_car    '3' // button left
#define right_car   '4' // button right
#define stop_car    '0' // button to stop
/******************** Car running status enumeration ********************/
enum {
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT
} enCarState;
int incomingByte = 0;          // received data byte
String inputString = "";         // Used to store received content
boolean newLineReceived = false; // end of previous data
boolean startBit  = false;  // agreement start sign
int g_carstate = enSTOP; //  1 before 2 after 3 left 4 right 0 stop
String returntemp = ""; // store return value
boolean g_autoup = false;
int g_uptimes = 5000;
int front = 0; // forward variable
int back = 0; // back variable
int turnl = 0; // left turn sign
int turnr = 0; // right turn sign
int spinl = 0; // Left rotation sign
int spinr = 0; // rotate right sign
int bluetoothvalue; // Bluetooth control amount
/******************** Ultrasonic speed ********************/
int chaoshengbo = 0;
int tingzhi = 0;
int jishi = 0;
/******************** Pulse calculation ********************/
void countpluse()
{
  lz = count_left;
  rz = count_right;
  count_left = 0;
  count_right = 0;
  lpluse = lz;
  rpluse = rz;
  if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0)) // Judging the direction of movement of the trolley When moving backward (PWM means the motor voltage is negative), the number of pulses is negative
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0)) // Judging the direction of movement of the trolley When moving forward (PWM means that the motor voltage is positive), the number of pulses is negative
  {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0)) // Judging the direction of movement of the trolley When moving forward (PWM means that the motor voltage is positive), the number of pulses is negative
  {
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0)) // Judging the movement direction of the trolley Left rotation The number of right pulses is negative and the number of left pulses is positive
  {
    rpluse = -rpluse;
    lpluse = lpluse;
  }
  // raise judgment
  balancecar.stopr += rpluse;
  balancecar.stopl += lpluse;
  // When entering the interrupt every 5ms, the number of pulses is superimposed
  balancecar.pulseright += rpluse;
  balancecar.pulseleft += lpluse;
  sumam = balancecar.pulseright + balancecar.pulseleft;
}
/******************** Angle PD ********************/
void angleout()
{
  balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x; // PD angle loop control
}
/******************** Interrupt timing 5ms timer interrupt ********************/
void inter()
{
  sei(); // open interrupt
  countpluse(); // Pulse Superposition Subfunction
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // IIC gets MPU6050 six-axis data ax ay az gx gy gz
  kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1); // Get angle angle and Kalman filter
  angleout(); // Angle Ring PD Control
  speedcc++;
  if (speedcc >= 10) // 50ms to enter speed loop control
  {
    Outputs = balancecar.speedpiout(kp_speed, ki_speed, kd_speed, front, back, setp0);
    speedcc = 0;
  }
  turncount++;
  if (turncount > 4) // 20ms into rotation control
  {
    turnoutput = balancecar.turnspin(turnl, turnr, spinl, spinr, kp_turn, kd_turn, kalmanfilter.Gyro_z); // Rotator function
    turncount = 0;
  }
  balancecar.posture++;
  balancecar.pwma(Outputs, turnoutput, kalmanfilter.angle, kalmanfilter.angle6, turnl, turnr, spinl,\
                  spinr, front, back, kalmanfilter.accelz, IN1M, IN2M, IN3M, IN4M, PWMA, PWMB); // Car total PWM output
}
/******************** Interrupt timing 5ms timing interrupt ********************/
void SendAutoUp()
{
  g_uptimes --;
  if ((g_autoup == true) && (g_uptimes == 0))
  {
    // automatic report
    String CSB, VT;
    char temp[10]={0};
    float fgx;
    float fay;
    float leftspeed;
    float rightspeed;
    fgx = gx; // pass to local variable
    fay = ay; // pass to local variable
    leftspeed = balancecar.pwm1;
    rightspeed = balancecar.pwm2;
    double Gx = (double)((fgx - 128.1f) / 131.0f); // angle conversion
    double Ay = ((double)fay / 16384.0f) * 9.8f;
   if(leftspeed > 255 || leftspeed < -255)
      return;
   if(rightspeed > 255 || rightspeed < -255)
      return;
   if((Ay < -20) || (Ay > 20))
      return;
   if((Gx < -3000) || (Gx > 3000))
      return;  
    returntemp = "";
    memset(temp, 0x00, sizeof(temp));
    // sprintf(temp, "%3.1f", leftspeed);
    dtostrf(leftspeed, 3, 1, temp);  // Equivalent to %3.2f
    String LV = temp;
    memset(temp, 0x00, sizeof(temp));
    // sprintf(temp, "%3.1f", rightspeed);
    dtostrf(rightspeed, 3, 1, temp);  // Equivalent to %3.1f
    String RV = temp;
    memset(temp, 0x00, sizeof(temp));
    // sprintf(temp, "%2.2f", Ay);
    dtostrf(Ay, 2, 2, temp);  // Equivalent to %2.2f
    String AC = temp;
    memset(temp, 0x00, sizeof(temp));
    // sprintf(temp, "%4.2f", Gx);
    dtostrf(Gx, 4, 2, temp);  // Equivalent to %4.2f
    String GY = temp;
    CSB = "0.00";
    VT = "0.00";
    // AC =
    returntemp = "$LV" + LV + ",RV" + RV + ",AC" + AC + ",GY" + GY + ",CSB" + CSB + ",VT" + VT + "#";
    Serial.print(returntemp); // return protocol packet
  }
  if (g_uptimes == 0)
      g_uptimes = 5000;
}
/******************** Initialize settings ********************/
void setup() {
  // TB6612FNGN driver module control signal initialization
  pinMode(IN1M, OUTPUT); // Control the direction of motor 1, 01 is forward rotation, 10 is reverse rotation
  pinMode(IN2M, OUTPUT);
  pinMode(IN3M, OUTPUT); // Control the direction of motor 2, 01 is forward rotation, 10 is reverse rotation
  pinMode(IN4M, OUTPUT);
  pinMode(PWMA, OUTPUT); // Left motor PWM
  pinMode(PWMB, OUTPUT); // Right motor PWM
  pinMode(STBY, OUTPUT); // enable TB6612FNG
  // Initialize the motor driver module
  digitalWrite(IN1M, 0);
  digitalWrite(IN2M, 1);
  digitalWrite(IN3M, 1);
  digitalWrite(IN4M, 0);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  // Speed ​​dial input
  pinMode(PinA_left, INPUT);  
  pinMode(PinA_right, INPUT);
  // Join the I2C bus
  Wire.begin(); // Join the I2C bus sequence
  Serial.begin(9600); // Open the serial port and set the baud rate to 115200
  delay(1500);
  mpu.initialize(); // Initialize MPU6050
  delay(2);
  balancecar.pwm1 = 0;
  balancecar.pwm2 = 0;
  // 5ms timer interrupt setting, use timer2. Note: using timer2 will affect the PWM output of pin3 pin11,
  // Because PWM uses a timer to control the duty cycle, when using a timer, pay attention to check the pin port of the corresponding timer.
  MsTimer2::set(5, inter);
  MsTimer2::start();
}
/******************** Reset PID parameters ********************/
void ResetPID()
{
  kp = PID_Original[0];
  ki =  PID_Original[1];
  kd =  PID_Original[2]; // Parameters you need to modify
  kp_speed =  PID_Original[3];
  ki_speed =  PID_Original[4];
  kd_speed =  PID_Original[5]; // Parameters you need to modify
}
/******************** reset cart status ********************/
void ResetCarState()
{
  turnl = 0; 
  turnr = 0;  
  front = 0; 
  back = 0; 
  spinl = 0; 
  spinr = 0; 
  turnoutput = 0;
}
/******************** main loop ********************/
void loop() 
{
  String returnstr = "$0,0,0,0,0,0,0,0,0,0,0,0cm,8.2V#"; // Send by default
  // In the main function, loop detection and superimposition of pulses Measure the speed of the car. Use level change to enter the pulse superposition. Increase the number of pulses of the motor to ensure the accuracy of the car.
  attachInterrupt(0, Code_left, CHANGE);
  attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);
  if (newLineReceived)
  {
    switch (inputString[1])
    {
      case run_car:   g_carstate = enRUN;   break;
      case back_car:  g_carstate = enBACK;  break;
      case left_car:  g_carstate = enLEFT;  break;
      case right_car: g_carstate = enRIGHT; break;
      case stop_car:  g_carstate = enSTOP;  break;
      default: g_carstate = enSTOP; break;
     }
    // Determine whether the protocol has packet loss
    if (inputString[3] == '1' && inputString.length() == 21)//左摇
    {
      g_carstate = enTLEFT;
      // Serial.print(returnstr);
      }
    else if (inputString[3] == '2' && inputString.length() == 21)//右摇
     {
      g_carstate = enTRIGHT;
      // Serial.print(returnstr);
     }
    if (inputString[5] == '1') // Query PID
      {
        char charkp[7], charkd[7], charkpspeed[7], charkispeed[7];
        dtostrf(kp, 3, 2, charkp); // Equivalent to %3.2f
        dtostrf(kd, 3, 2, charkd); // Equivalent to %3.2f
        dtostrf(kp_speed, 3, 2, charkpspeed); // Equivalent to %3.2f
        dtostrf(ki_speed, 3, 2, charkispeed); // Equivalent to %3.2f
        String strkp = charkp; String strkd = charkd; 
        String strkpspeed = charkpspeed; String strkispeed = charkispeed;
        returntemp = "$0,0,0,0,0,0,AP" + strkp + ",AD" + strkd + ",VP" + strkpspeed + ",VI" + strkispeed + "#";
        Serial.print(returntemp); // return protocol packet
      }
      else if (inputString[5] == '2') // restore PID
      {
        ResetPID();
        Serial.print("$OK#"); // return protocol packet
      }
      if (inputString[7] == '1') // automatic report
      {
        g_autoup = true;
        Serial.print("$OK#"); // return protocol packet
      }
      else if (inputString[7] == '2') // Stop automatic reporting
      {
        g_autoup = false;
        Serial.print("$OK#"); // return protocol packet
      }
      if (inputString[9] == '1') // Angle Ring Update $0,0,0,0,1,1,AP23.54,AD85.45,VP10.78,VI0.26#
      {
        int i = inputString.indexOf("AP");
        int ii = inputString.indexOf(",", i);
        if(ii > i)
        {
          String m_skp = inputString.substring(i + 2, ii);
          m_skp.replace(".", "");
          int m_kp = m_skp.toInt();
          kp = (double)( (double)m_kp / 100.0f);
        }
        i = inputString.indexOf("AD");
        ii = inputString.indexOf(",", i);
        if(ii > i)
        {
          // ki = inputString.substring(i+2, ii);
          String m_skd = inputString.substring(i + 2, ii);
          m_skd.replace(".", "");
          int m_kd = m_skd.toInt();
          kd = (double)( (double)m_kd / 100.0f);
        }
        Serial.print("$OK#"); // return protocol packet
      }
      if (inputString[11] == '1') // Speed ​​Ring Update
      {
        int i = inputString.indexOf("VP");
        int ii = inputString.indexOf(",", i);
        if(ii > i)
        {
          String m_svp = inputString.substring(i + 2, ii);
          m_svp.replace(".", "");
          int m_vp = m_svp.toInt();
          kp_speed = (double)( (double)m_vp / 100.0f);
        }
        i = inputString.indexOf("VI");
        ii = inputString.indexOf("#", i);
        if(ii > i)
        {
          String m_svi = inputString.substring(i + 2, ii);
          m_svi.replace(".", "");
          int m_vi = m_svi.toInt();
          ki_speed = (double)( (double)m_vi / 100.0f);
          Serial.print("$OK#"); // return protocol packet
        }
      }
      // reset
      inputString = "";   // clear the string
      newLineReceived = false;
    } 
a:    switch (g_carstate)
    {
      case enSTOP:turnl = 0;turnr = 0;front = 0;back = 0;spinl = 0;spinr = 0;turnoutput = 0;break;
      case enRUN:ResetCarState();front = 250;break;
      case enLEFT:turnl = 1;break;
      case enRIGHT:turnr = 1;break;
      case enBACK:ResetCarState();back = -250;break;
      case enTLEFT:spinl = 1;break;
      case enTRIGHT:spinr = 1;break;
      default:front = 0;back = 0;turnl = 0;turnr = 0;spinl = 0;spinr = 0;turnoutput = 0;break;
    }
   // Add automatic reporting
  SendAutoUp();
}
/******************** Pulse interrupt service function ********************/
/******************** Left tacho code wheel count ********************/
void Code_left() 
{
  count_left ++;
} 
/******************** Right tachometer count ********************/
void Code_right() 
{
  count_right ++;
}
//serialEvent() It is a new function of IDE1.0 and later versions. It is not clear why most people do not want to use it. This is equivalent to the interrupt function!
int num1 = 0;
void serialEvent()
{
  while (Serial.available())
  {
    incomingByte = Serial.read(); // Read byte by byte, the next sentence is read and put into the string array to form a completed packet
    if (incomingByte == '$')
    {
      num1 = 0;
      startBit = true;
    }
    if (startBit == true)
    {
      num1++;
      inputString += (char) incomingByte; // The full-duplex serial port does not need to add a delay below, and half-duplex needs to be added//
    }
    if (startBit == true && incomingByte == '#')
    {
      newLineReceived = true;
      startBit = false;
    }
    
    if(num1 >= 80)
    {
      num1 = 0;
      startBit = false;
      newLineReceived = false;
      inputString = "";
    }
  }
}
/******************** backup ********************/
#if 0
char chartemp[7];
dtostrf(ax, 3, 2, chartemp); // Equivalent to %3.2f
String strax = chartemp;
strax = "\nax:" + strax;

memset(chartemp, 0x00, 7);
dtostrf(ay, 3, 2, chartemp); // Equivalent to %3.2f
String stray = chartemp;
stray = "\nay:" + stray;

memset(chartemp, 0x00, 7);
dtostrf(az, 3, 2, chartemp); // Equivalent to %3.2f
String straz = chartemp;
straz = "\naz:" + straz;

memset(chartemp, 0x00, 7);
dtostrf(gx, 3, 2, chartemp); // Equivalent to %3.2f
String strgx = chartemp;
strgx = "\ngx:" + strgx;

memset(chartemp, 0x00, 7);
dtostrf(gy, 3, 2, chartemp); // Equivalent to %3.2f
String strgy = chartemp;
strgy = "\ngy:" + strgy;

memset(chartemp, 0x00, 7);
dtostrf(gz, 3, 2, chartemp); // Equivalent to %3.2f
String strgz = chartemp;
strgz = "\ngz:" + strgz;
Serial.print(strax + stray + straz + strgx + strgy + strgz); // return protocol packet
#endif
