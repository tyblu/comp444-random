#include "inttypes.h"

//#define DEBUG
#ifdef DEBUG
# define PRE Serial.print(F("temp_ctrl: "))
# define POST delay(2)
# define DEBUG0(x) (x)
# define DEBUG1(x) PRE; Serial.println(x); POST
# define DEBUG10(x) Serial.print(x); POST
#else
# define DEBUG0(x)
# define DEBUG1(x)
# define DEBUG10(x)
#endif

#define TEMP_ADC_PIN A3
#define HEATER_PIN 8

#define SAMPLE_DELAY_MS 1000L
#define HEATER_PERIOD 10000L

#define SET_POINT_MODE
//#define P_MODE
//#define PI_MODE
//#define PD_MODE
//#define PID_MODE

#ifdef SET_POINT_MODE
#	define DUTY_INIT 0.0
#else
#	define DUTY_INIT 0.0
#endif

#define PGAIN 1
#define IGAIN 0
#define DGAIN 0
#define IMIN 0
#define IMAX 100

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

#define TEMPERATURE_DATA_POINT_COUNT 10
float getTemperature()
{
  uint32_t sum = 0;
  for (uint8_t i = 0; i<TEMPERATURE_DATA_POINT_COUNT; i++)
    sum += analogRead(TEMP_ADC_PIN);
	return ( sum * 0.0048828125 / (float)TEMPERATURE_DATA_POINT_COUNT - 0.5 ) * 100.0 + 5.0;
}

void heaterOn()
{
	digitalWrite(HEATER_PIN, HIGH);
  DEBUG1(F("Heater turned on."));
}

void heaterOff()
{
	digitalWrite(HEATER_PIN, LOW);
}

float temp, tempSetPoint;
float heaterDutyCycle;
uint32_t timeZero, sampleTime, heaterOnTime, heaterPeriodTime;
long sampleTimeLeft, heaterOnTimeLeft, heaterPeriodTimeLeft;
SPid pid;
float pidDrive;

const char comma PROGMEM = ',';
#define printCsv(x) Serial.print(x); Serial.write(comma)
void printData(SPid * pid)
{
	printCsv((millis() - timeZero) / 1000UL);
	printCsv(temp);
	printCsv(tempSetPoint);
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

	temp = getTemperature();
	pinMode(HEATER_PIN, OUTPUT);
	heaterOn();

	pid.init();
	pidDrive = updatePID(&pid, 0, temp);
#ifdef DUTY_INIT
	heaterDutyCycle = DUTY_INIT;
#else
	heaterDutyCycle = pidDrive / 100;
#endif

	timeZero = millis();
	sampleTime = millis();			// sample (etc) immediately
	heaterPeriodTime = millis();	// start new period immediately
}

#ifdef SET_POINT_MODE
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
		heaterOnTime = millis() + (uint32_t)(heaterDutyCycle * HEATER_PERIOD);
		heaterPeriodTime = millis() + HEATER_PERIOD;
    
    DEBUG0(Serial.print(F("heaterOnTime - millis() = ")));
    DEBUG0(Serial.println(heaterOnTime - millis()));
    DEBUG0(Serial.print(F("heaterPeriodTime - ...  = ")));
    DEBUG0(Serial.println(heaterPeriodTime - millis()));
	}

	delay(100);
}
#endif // SET_POINT_MODE

#ifdef P_MODE
void loop()
{
	//
}
#endif // P_MODE

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
	heaterToggleTimeLeft = heaterToggleTime - millis();
	sampleTimeLeft = sampleTime - millis();

	if (heaterToggleTimeLeft < 0)
	{
		heaterToggleCount++;
		if (heaterDutyCycle * heaterToggleCount > 0.95)
		{
			heaterOn();
			heaterToggleCount = 0;
		}
		else
			heaterOff();

		heaterToggleTime = millis() + HEATER_TOGGLE_DELAY_MS;
	}

	if (sampleTimeLeft < 0)
	{
		temp = getTemperature();
		drive = updatePID(&pid, tempSetPoint - temp, temp);

		if (drive > DRIVE_MAX)
			heaterDutyCycle = 1.0;
		else if (drive > DRIVE_MIN)
			heaterDutyCycle = drive / 100;
		else
			heaterDutyCycle = 0.0;

		sampleTime = millis() + SAMPLE_DELAY_MS;

		// ...
	}
}
#endif // PID_MODE
