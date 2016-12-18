// attiny_i2c_sonar
// by
// Andrew Kramer

// measures distance using a sonar sensor and reports the distance via i2c 

#include <TinyWireS.h>
#include <TinyWireM.h>
#include <Arduino.h>
#include <AttinyPing.h>

#define OWN_ADDRESS       0x4
#define MASTER_ADDRESS    0x6
#define NUM_SONAR           8
#define SONAR0              1
#define SONAR1              2
#define SONAR2              3
#define SONAR3              5
#define SONAR4              7
#define SONAR5              8
#define SONAR6              9
#define SONAR7             10
#define MAX_DISTANCE      400
#define PING_INTERVAL     250

int pingTimes[NUM_SONAR];
uint8_t index;
long lastTime;
const double speedOfSound = .0343; // in cm/microsecond

AttinyPing sonar[NUM_SONAR] = {
  AttinyPing(SONAR0, SONAR0, MAX_DISTANCE),
  AttinyPing(SONAR1, SONAR1, MAX_DISTANCE),
  AttinyPing(SONAR2, SONAR2, MAX_DISTANCE),
  AttinyPing(SONAR3, SONAR3, MAX_DISTANCE),
  AttinyPing(SONAR4, SONAR4, MAX_DISTANCE),
  AttinyPing(SONAR5, SONAR5, MAX_DISTANCE),
  AttinyPing(SONAR6, SONAR6, MAX_DISTANCE),
  AttinyPing(SONAR7, SONAR7, MAX_DISTANCE),
};


void setup()
{
	TinyWireS.begin(OWN_ADDRESS);
	TinyWireS.onReceive(updatePingTimes);
	lastTime = 0;
  index = 0;
}

void loop()
{
  TinyWireS_stop_check();
}

void updatePingTimes(uint8_t bytes)
{
  TinyWireS.receive();
  for (int i = 0; i < NUM_SONAR; i++)
  {
    pingTimes[i] = sonar[i].ping();
  }
  sendTimes();
}

void sendTimes()
{
  USICR = 0;
  USISR = 0;
  USIDR = 0;
  TinyWireM.begin();
  TinyWireM.beginTransmission(MASTER_ADDRESS);
  for (int i = 0; i < NUM_SONAR; i++)
  {
    int thisTime = pingTimes[i];
    if (thisTime == 0) thisTime = (double)MAX_DISTANCE / speedOfSound;
    uint8_t firstByte = thisTime & 0xFF;
    uint8_t secondByte = (thisTime >> 8) & 0xFF;
    TinyWireM.write(firstByte);
    TinyWireM.write(secondByte);
  }
  TinyWireM.endTransmission();
  USICR = 0;
  USISR = 0;
  USIDR = 0;
  TinyWireS.begin(OWN_ADDRESS);
}
