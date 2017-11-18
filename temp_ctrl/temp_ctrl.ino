#include "inttypes.h"

//#define DEBUG
#ifdef DEBUG
# define PRE Serial.print(F("temp_ctrl: "))
# define POST delay(2)
# define DEBUG0(x) (x)
# define DEBUG1(x) PRE; Serial.println(x); POST
# define DEBUG10(x) Serial.print(x); POST
# define DEBUG2(x,y) PRE; Serial.print(x); Serial.println(y); POST
# define DEBUG20(x,y) PRE; Serial.print(x); Serial.print(y); POST
#else
# define DEBUG0(x)
# define DEBUG1(x)
# define DEBUG10(x)
# define DEBUG2(x,y)
# define DEBUG20(x,y)
#endif

#define TEMP_ADC_PIN A3
#define HEATER_PIN 8

#define SAMPLE_DELAY_MS 250L
#define HEATER_PERIOD_MS 10000L
#define PID_UPDATE_PERIOD_MS 125L

//#define OPEN_LOOP_MODE
#define P_MODE
//#define I_MODE
//#define PI_MODE
//#define PD_MODE
//#define PID_MODE

#	define DUTY_INIT 0.0

#define PGAIN 0.1
#define IGAIN 0
#define DGAIN 0
#define IMIN 0
#define IMAX 100

#define TARGET 23.5 // 21.5C ambient + 23.5C rise = 45C target

#define TEMP_TO_DUTY_SLOPE 0.1
#define TEMP_TO_DUTY_OFFSET 0.0

#define DRIVE_MAX 100
#define DRIVE_MIN 10

struct SPid
{
	float dState;
	float iState;
	float iMax, iMin;

	float pGain, iGain, dGain;

	void init()
	{
		pGain = PGAIN;
		iGain = IGAIN;
		dGain = DGAIN;
		iMin = IMIN;
		iMax = IMAX;
		dState = 0;
		iState = 0;
	}
};

float temp, tempRiseTarget, tempZero;
float heaterDutyCycle;
uint32_t timeZero, sampleTime, heaterOnTime, heaterPeriodTime, pidUpdateTime;
long sampleTimeLeft, heaterOnTimeLeft, heaterPeriodTimeLeft, pidUpdateTimeLeft;
SPid pid;
float pidDrive;
bool heaterIsOn;

float updatePID(SPid * pid, float err, float pos)
{
	float pTerm, dTerm, iTerm;

	pTerm = pid->pGain * err;
	
	pid->iState += err;
	if (pid->iState > pid->iMax)
		pid->iState = pid->iMax;
	if (pid->iState < pid->iMin)
		pid->iState = pid->iMin;

	iTerm = pid->iGain * pid->iState;

	dTerm = pid->dGain * (pid->dState - pos);
	pid->dState = pos;

	return pTerm + dTerm + iTerm;
}

#define TEMPERATURE_DATA_POINT_COUNT 100
float getTemperature()
{
  uint32_t sum = 0;
  for (uint8_t i = 0; i<TEMPERATURE_DATA_POINT_COUNT; i++)
    sum += analogRead(TEMP_ADC_PIN);
	return ( sum * 0.0048828125 / (float)TEMPERATURE_DATA_POINT_COUNT - 0.5 ) * 100.0 + 5.0;
}

#define TEMPZERO_DATA_POINT_COUNT 100
#define TEMPZERO_DELAY_MS 20
float getTempZero()
{
  float sum = 0;
  for (uint8_t i = 0; i < TEMPZERO_DATA_POINT_COUNT; i++)
  {
    delay(TEMPZERO_DELAY_MS);
    sum += getTemperature();
  }

#ifdef DEBUG
  float result = sum / (float)TEMPZERO_DATA_POINT_COUNT;
  Serial.print(F("Baseline temperature: "));
  Serial.print(result);
  Serial.println(F(" degrees Celsius"));
  return result;
#else
  return sum / (float)TEMPZERO_DATA_POINT_COUNT;
#endif
}

float tempChangeToDuty(float tempChange)
{
  return min(1.0, max(0.0, tempChange * TEMP_TO_DUTY_SLOPE + TEMP_TO_DUTY_OFFSET));
}

void heaterOn()
{
	digitalWrite(HEATER_PIN, HIGH);
  heaterIsOn = true;
  DEBUG1(F("Heater turned on."));
}

void heaterOff()
{
	digitalWrite(HEATER_PIN, LOW);
  heaterIsOn = false;
}

const char comma PROGMEM = ',';
#define printCsv(x) Serial.print(x); Serial.write(comma)
void printData(SPid * pid)
{
	printCsv( (millis() - timeZero) / (float)1000 );
	printCsv(temp);
	printCsv(tempRiseTarget);
	printCsv(heaterDutyCycle);
	printCsv(pid->pGain);
	printCsv(pid->iGain);
	printCsv(pid->dGain);
	Serial.println();
}

void printDataHeader()
{
	printCsv(F("time [sec]"));
	printCsv(F("temperature [0-255]"));
	printCsv(F("set point"));
	printCsv(F("heater duty cycle"));
	printCsv(F("pGain"));
	printCsv(F("iGain"));
	printCsv(F("dGain"));
	Serial.println();
}

void setup()
{
	Serial.begin(9600);
	Serial.println(F("temp_ctrl.ino compiled " __DATE__ " at " __TIME__));
	Serial.println();
	printDataHeader();

  pinMode(HEATER_PIN, OUTPUT);
  heaterOff();
  
//  tempZero = 20.5;
  tempZero = getTempZero();
	temp = getTemperature();
  tempRiseTarget = TARGET;

	pid.init();
	pidDrive = updatePID(&pid, tempZero + tempRiseTarget - temp, temp);
	heaterDutyCycle = DUTY_INIT;

	timeZero = millis();
	sampleTime = millis();			// sample (etc) immediately
	heaterPeriodTime = millis();	// start new period immediately
}

#ifdef OPEN_LOOP_MODE
void loop()
{
	sampleTimeLeft = sampleTime - millis();
	heaterOnTimeLeft = heaterOnTime - millis();
	heaterPeriodTimeLeft = heaterPeriodTime - millis();

	if (sampleTimeLeft < 0)
	{
		temp = getTemperature();
		printData(&pid);
		sampleTime = millis() + SAMPLE_DELAY_MS;
	}

	if (heaterOnTimeLeft < 0)
		heaterOff();

	if (heaterPeriodTimeLeft < 0)
	{
		heaterOn();
		heaterOnTime = millis() + (uint32_t)(heaterDutyCycle * HEATER_PERIOD_MS);
		heaterPeriodTime = millis() + HEATER_PERIOD_MS;
    
    DEBUG0(Serial.print(F("heaterOnTime - millis() = ")));
    DEBUG0(Serial.println(heaterOnTime - millis()));
    DEBUG0(Serial.print(F("heaterPeriodTime - ...  = ")));
    DEBUG0(Serial.println(heaterPeriodTime - millis()));
	}

	delay(50);
}
#endif // OPEN_LOOP_MODE

#ifdef P_MODE
void loop()
{
  sampleTimeLeft = sampleTime - millis();
  pidUpdateTimeLeft = pidUpdateTime - millis();
  heaterOnTimeLeft = heaterOnTime - millis();
  heaterPeriodTimeLeft = heaterPeriodTime - millis();

  if (sampleTimeLeft < 0)
  {
    temp = getTemperature();
    printData(&pid);
    sampleTime = millis() + SAMPLE_DELAY_MS;
  }

  if (heaterOnTimeLeft < 0 && heaterIsOn)
    heaterOff();

  if (heaterPeriodTimeLeft < 0)
  {
    heaterOn();
    heaterOnTime = millis() + (uint32_t)(heaterDutyCycle * HEATER_PERIOD_MS);
    heaterPeriodTime = millis() + HEATER_PERIOD_MS;
  }

  if (pidUpdateTimeLeft < 0)
  {
    pidDrive = updatePID(&pid, tempZero + tempRiseTarget - temp, temp - tempZero);
    heaterDutyCycle = tempChangeToDuty(pidDrive);
    pidUpdateTime = millis() + PID_UPDATE_PERIOD_MS;
    
    DEBUG2(F("pidDrive        = "), pidDrive);
    DEBUG2(F("heaterDutyCycle = "), heaterDutyCycle);
  }
}
#endif // P_MODE

#ifdef I_MODE
void loop()
{
  sampleTimeLeft = sampleTime - millis();
  pidUpdateTimeLeft = pidUpdateTime - millis();
  heaterOnTimeLeft = heaterOnTime - millis();
  heaterPeriodTimeLeft = heaterPeriodTime - millis();

  if (sampleTimeLeft < 0)
  {
    temp = getTemperature();
    printData(&pid);
    sampleTime = millis() + SAMPLE_DELAY_MS;
  }

  if (heaterOnTimeLeft < 0 && heaterIsOn)
    heaterOff();

  if (heaterPeriodTimeLeft < 0)
  {
    heaterOn();
    heaterOnTime = millis() + (uint32_t)(heaterDutyCycle * HEATER_PERIOD_MS);
    heaterPeriodTime = millis() + HEATER_PERIOD_MS;
  }

  if (pidUpdateTimeLeft < 0)
  {
    pidDrive = updatePID(&pid, tempZero + tempRiseTarget - temp, temp - tempZero);
    heaterDutyCycle = tempChangeToDuty(pidDrive);
    pidUpdateTime = millis() + PID_UPDATE_PERIOD_MS;
    
    DEBUG2(F("pidDrive        = "), pidDrive);
    DEBUG2(F("heaterDutyCycle = "), heaterDutyCycle);
  }
}
#endif // I_MODE

#ifdef PI_MODE
void loop()
{
	//
}
#endif // PI_MODE

#ifdef PD_MODE
void loop()
{
	//
}
#endif // PD_MODE

#ifdef PID_MODE
void loop()
{
  //
}
#endif // PID_MODE
