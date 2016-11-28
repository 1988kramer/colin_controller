#include <DifferentialDrive.h>
#include <Colin.h>
#include <TimerOne.h>

const char SOP = '<';
const char EOP = '>';
const char DEL = ',';

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
  printPose();
}

void printParameters(double *parameters)
{
  Serial.print("executing command \ntranslational: ");
  Serial.print((int)parameters[0]);
  Serial.print(" mm/s \nangular: ");
  Serial.print(parameters[1]);
  Serial.print(" rad/s \ntime: ");
  Serial.print((int)parameters[2]);
  Serial.println(" sec");
  Serial.println();
}

void printPose()
{
  colin.getPosition(x, y, theta);
  Serial.print(SOP);
  Serial.print((int)x, DEC);
  Serial.print(DEL);
  Serial.print((int)y, DEC);
  Serial.print(DEL);
  Serial.print(theta, 3);
  Serial.print(EOP);
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

