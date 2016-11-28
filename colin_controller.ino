#include <DifferentialDrive.h>
#include <Colin.h>
#include <TimerOne.h>
#include <Wire.h>

#define SONAR_ADDRESS        0x8
#define OWN_ADDRESS          0x6
#define NUM_SONAR              8
#define SONAR_PER_CONTROLLER   8
#define NUM_CONTROLLERS        1

int sonarDistances[NUM_SONAR]; // array of ping times from sonar sensors
const uint8_t x = 1; // meaningless value to trigger update
const double speedOfSound = 0.0343; // in cm/microsecond
bool distancesRead = false;

const char SOP = '<'; // designates start of packet
const char EOP = '>'; // designates end of packet
const char DEL = ','; // delimits between values
const char SON = 'S'; // designates sonar packet
const char ODT = 'O'; // designates odometry packet

bool started = false;
bool ended = false;

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
int index = 0;


void setup() {
  Serial.begin(9600); // start serial with Raspberry Pi
  
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
}


void loop() 
{
  readCommandPacket();
}

void readCommandPacket()
{
  char commandPacket[32];
  while (Serial.available() > 0)
  {
    char inChar = Serial.read();
    if (!started)
    {
      if (inChar == SOP)
      {
        index = 0;
        commandPacket[index] = '\0';
        started = true;
        ended = false;
      }
    }
    else if (!ended)
    {
      if (inChar == EOP)
      {
        ended = true;
      }
      else
      {
        commandPacket[index] = inChar;
        index++;
        commandPacket[index] = '\0';
      }
    }
  }

  if (started && ended)
  {
    parseCommandPacket(commandPacket);
  }
}

void parseCommandPacket(char *commandPacket)
{
  int index = 0;
  double parameters[3];
  for (int i = 0; i < 3; i++)
  {
    char buf[10];
    memset(buf, '\0', sizeof(buf));
    char bufIndex = 0;
    while (commandPacket[index] != DEL && commandPacket[index] != '\0') 
    {
      buf[bufIndex] = commandPacket[index];
      bufIndex++;
      index++;
    }
    parameters[i] = atof(buf);
    index++;
    bufIndex = 0;
    memset(buf, '\0', sizeof(buf));
  }
  started = false;
  ended = false;
  colin.drive((int)parameters[0], parameters[1]);
  delay((int)parameters[2]);
  colin.drive(0, 0.0);
  delay(250);
  sendPose();
  requestSonarUpdate();
  while(!distancesRead);
  sendDistances();
}

// sends pose from odometry to raspberry pi
void sendPose()
{
  colin.getPosition(x, y, theta);
  Serial.print(SOP);
  Serial.print(ODT);
  Serial.print(DEL);
  Serial.print((int)x, DEC);
  Serial.print(DEL);
  Serial.print((int)y, DEC);
  Serial.print(DEL);
  Serial.print(theta, 3);
  Serial.print(EOP);
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
  sonarDistances[index] = ((secondByte << 8) | firstByte) * speedOfSound;
}

// send updated sonar distance array to the raspberry pi
void sendDistances()
{
  Serial.print(SOP);
  Serial.print(SON);
  for (int i = 0; i < NUM_SONAR; i++)
  {
    Serial.print(DEL);
    Serial.print(sonarDistances[i]);
  }
  Serial.print(EOP);
}

void requestSonarUpdate(int index)
{
  distancesRead = false;
  Wire.beginTransmission(SONAR_ADDRESS);
  Wire.write(x);
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

