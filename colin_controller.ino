#include <DifferentialDrive.h>
#include <Colin.h>
#include <TimerOne.h>
#include <Wire.h>

#define SONAR_ADDRESS        0x4
#define OWN_ADDRESS          0x6
#define NUM_SONAR              8
#define SONAR_PER_CONTROLLER   8
#define NUM_CONTROLLERS        1

int sonarDistances[NUM_SONAR]; // array of ping times from sonar sensors
const uint8_t trig = 1; // meaningless value to trigger update
const double speedOfSound = 0.0343; // in cm/microsecond
bool distancesRead;
bool commandReceived;

const char SOP = '<'; // designates start of packet
const char EOP = '>'; // designates end of packet
const char DEL = ','; // delimits between values

Motor rhMotor(RH_DIR1, RH_DIR2, RH_PWM);
Encoder rhEncoder(RH_ENCODER_A, RH_ENCODER_B, deltaT, ticksPerRev);
SpeedControl rhSpeedControl(&rhMotor, &rhEncoder);
PositionControl rhPosition(&rhSpeedControl);

Motor lhMotor(LH_DIR1, LH_DIR2, LH_PWM);
Encoder lhEncoder(LH_ENCODER_A, LH_ENCODER_B, deltaT, ticksPerRev);
SpeedControl lhSpeedControl(&lhMotor, &lhEncoder);
PositionControl lhPosition(&lhSpeedControl);

DifferentialDrive colin(&lhPosition, &rhPosition, 
                        wheelCirc, wheelDist);

double x, y; 
double theta;
unsigned long lastCommandTime, currentTime;

// for testing purposes only
int translational;
double angular;


void setup() {
  Serial.begin(9600); // start serial with Raspberry Pi
  Serial.setTimeout(100);
  
  // start timer and hardware interrupts
  Timer1.initialize(deltaT);
  Timer1.attachInterrupt(adjust);
  attachInterrupt(0, readLHEncoder, CHANGE);
  attachInterrupt(1, readRHEncoder, CHANGE);

  // set PID gains for each motor
  rhSpeedControl.setGains(kP, kI, kD);
  lhSpeedControl.setGains(kP, kI, kD);
  
  // begin communication with sonar controller
  Wire.begin(OWN_ADDRESS);
  Wire.onReceive(updateDistances);
  
  distancesRead = false;
  commandReceived = false;

  lastCommandTime = millis();
  currentTime = millis();
  
  translational = 0;
  angular = 0.0;
}


void loop() 
{ 
  // check if a command packet is available to read
  readCommandPacket();
  
  // request a sensor update if a command has been received
  if (commandReceived)
  {
    commandReceived = false;
    requestSonarUpdate(SONAR_ADDRESS);
  }
  // send sensor packet if sonar has finished updating
  if (distancesRead)
  {
    distancesRead = false; 
    sendSensorPacket();
  }
  currentTime = millis();
  // stop colin if a command packet has not been received for 1 second
  if (currentTime - lastCommandTime > 1000)
  {
    Serial.println("command not received for 1 second");
    lastCommandTime = millis();
    colin.drive(0, 0.0);
  }
}

void readCommandPacket()
{
  byte buffer[4];
  int result = Serial.readBytes((char*)buffer, 4);
  int commands[2];
  
  for (int i = 0; i < 2; i++)
  {
    int firstByte = buffer[2 * i];
    int secondByte = buffer[(2 * i) + 1];
    commands[i] = (secondByte << 8) | firstByte;
  }
  
  if (result == 4) // if the correct number of bytes has been received
  {
    translational = commands[0];
    angular = (double)commands[1] / 1000.0;
    colin.drive(translational, angular);
    commandReceived = true;
    lastCommandTime = millis();
  }
  else if (result > 0)
  {
    Serial.println("incomplete command");
  }
  // else do nothing and try again later
}

// sends pose from odometry to raspberry pi
void sendSensorPacket()
{
  /*
  Serial.print("speeds ");
  Serial.print(translational);
  Serial.print(" ");
  Serial.print(angular);
  Serial.print(" ");
  */
  colin.getPosition(x, y, theta);
  byte buffer[22];
  addDistances(buffer);
  int sendX = (int)x;
  int sendY = (int)y;
  int sendTheta = (int)(theta * 1000.0);
  buffer[16] = (byte)(sendX & 0xFF);
  buffer[17] = (byte)((sendX >> 8) & 0xFF);
  buffer[18] = (byte)(sendY & 0xFF);
  buffer[19] = (byte)((sendY >> 8) & 0xFF);
  buffer[20] = (byte)(sendTheta & 0xFF);
  buffer[21] = (byte)((sendTheta >> 8) & 0xFF);
  Serial.write(buffer, 22);
}

void updateDistances(int bytes)
{
  for (int i = 0; i < NUM_SONAR; i++)
    readSonar(i);
  distancesRead = true;
}

// read distances from sonar
void readSonar(int index)
{
  int firstByte = Wire.read();
  int secondByte = Wire.read();
  sonarDistances[index] = ((double)((secondByte << 8) | firstByte)) * speedOfSound * 0.5;
}

// send updated sonar distance array to the raspberry pi
void addDistances(byte* buffer)
{
  for (int i = 0; i < NUM_SONAR; i++)
  {
    buffer[2 * i] = (byte)(sonarDistances[i] & 0xFF);
    buffer[(2 * i) + 1] = (byte)((sonarDistances[i] >> 8) & 0xFF);
  }
}

void requestSonarUpdate(int address)
{
  Wire.beginTransmission(address);
  Wire.write(trig);
  Wire.endTransmission();
}

void readLHEncoder()
{
  lhEncoder.updateCount();
}

void readRHEncoder()
{
  rhEncoder.updateCount();
}

void adjust()
{
  colin.update();
}

