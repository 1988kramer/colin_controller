// PID_tune.ino
// by Andrew Kramer
// 12/16/2016

// assists in manual PID tuning of Colin the Robot
// must be used with PID_tune.cpp running on the Raspberry Pi

#include <DifferentialDrive.h>
#include <Colin.h>
#include <TimerOne.h>

Motor rhMotor(RH_DIR1, RH_DIR2, RH_PWM);
Encoder rhEncoder(RH_ENCODER_A, RH_ENCODER_B, deltaT, ticksPerRev);
SpeedControl rhSpeedControl(&rhMotor, &rhEncoder);
PositionControl rhPosition(&rhSpeedControl);

Motor lhMotor(LH_DIR1, LH_DIR2, LH_PWM);
Encoder lhEncoder(LH_ENCODER_A, LH_ENCODER_B, deltaT, ticksPerRev);
SpeedControl lhSpeedControl(&lhMotor, &lhEncoder);
PositionControl lhPosition(&lhSpeedControl);

DifferentialDrive colin(&lhPosition, &rhPosition, wheelCirc, wheelDist);
												
const char ACK = 'a';
const int commandPacketLength = 12;
												
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
}

void loop()
{
	int commands[commandPacketLength / 2];
	
	if (readCommandPacket(commands) == 1)
	{
		sendACK();
		executeCommands(commandPacket);
	}
}

int readCommandPacket(int* commands)
{
	char commandPacket[commandPacketLength];
	int result = Serial.readBytes(commandPacket, commandPacketLength);
	
	if (result == commandPacketLength)
	{
		for (int i = 0; i < commandPacketLength / 2; i++)
		{
			int firtByte = commandPacket[2 * i];
			int secondByte = commandPacket[(2 * i) + 1);
			commands[i] = (secondByte << 8) | firstByte;
		}
		return 1;
	}
	else
	{
		return -1;
	}
}

void executeCommands(int* commands)
{
	double newKP = ((double)commands[3]) / 10000.0;
	double newKI = ((double)commands[4]) / 10000.0;
	double newkD = ((double)commands[5]) / 10000.0;
	
	// set new PID gains for both motors
	rhSpeedControl.setGains(newKP, newKI, newKD);
	lsSpeedControl.setGains(newKP, newKI, newKD);
	
	// set speed for the specified time 
	double angular = ((double)commands[1]) / 10000.0;
	colin.drive(commands[0], angular);
	delay(commands[2]);
	colin.drive(0, 0.0);
}

void sendACK()
{
	Serial.print(ACK);
}