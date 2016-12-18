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
  char commandPacket[32];
  memset(commandPacket, '\0', 32);
  bool started = false;
  bool ended = false;
  int index = 0;
  int result = Serial.readBytes(commandPacket,32);
  if (result > 0)
  {
    parseCommandPacket(commandPacket);
  }
}

void parseCommandPacket(char *commandPacket)
{
  bool started = false;
  bool ended = false;
  int packetIndex = 0;
  int speeds[2];
  int speedsIndex = 0;
  while (!started && packetIndex < 32)
  {
    if (commandPacket[packetIndex] == SOP)
      started = true;
    packetIndex++;
  }
  
  int bufferSize = 10;
  char buffer[bufferSize];
  memset(buffer, '\0', bufferSize);
  int bufferIndex = 0;
  
  while (!ended && packetIndex < 32 && speedsIndex < 2)
  {
    if (commandPacket[packetIndex] == DEL)
    {
      bufferIndex = 0;
      speeds[speedsIndex] = atoi(buffer);
      memset(buffer, '\0', bufferSize);
      speedsIndex++;
    }
    else if (commandPacket[packetIndex] == EOP)
    {
      ended = true;
      speeds[speedsIndex] = atoi(buffer);
    }
    else
    {
      buffer[bufferIndex] = commandPacket[packetIndex];
      bufferIndex++;
    }
    packetIndex++;
  }
  
  if (started && ended)
  {
    translational = speeds[0];
    angular = ((double)speeds[1]) / 1000.0;
    colin.drive(translational, angular);
    commandReceived = true;
    lastCommandTime = millis();
  }
  else
  {
    Serial.println("Bad command packet");
  }
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
  sendDistances();
  Serial.print((int)x);
  Serial.print(DEL);
  Serial.print((int)y);
  Serial.print(DEL);
  Serial.print((int)(theta * 1000));
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
  sonarDistances[index] = ((double)((secondByte << 8) | firstByte)) * speedOfSound * 0.5;
}

// send updated sonar distance array to the raspberry pi
void sendDistances()
{
  Serial.print(SOP);
  for (int i = 0; i < NUM_SONAR; i++)
  {
    Serial.print(sonarDistances[i]);
    Serial.print(DEL);
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

