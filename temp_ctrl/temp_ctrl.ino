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

#define SAMPLE_DELAY_MS 50L
#define PRINT_DELAY_MS 3000L
#define HEATER_PERIOD_MS 1500L
#define PID_UPDATE_PERIOD_MS 15000L

//#define OPEN_LOOP_MODE
#define PID_MODE

#	define DUTY_INIT 0.0

#define PGAIN 2
#define IGAIN 0.1
#define DGAIN 0
#define IMIN -25    // IMIN = iTerm min/IGAIN
#define IMAX 25     // IMAX = iTerm max/IGAIN

#define TARGET 23.5 // ~20.5C ambient + 23.5C rise = 44C'ish

#define TEMP_TO_DUTY_SLOPE 0.1
#define TEMP_TO_DUTY_OFFSET 0.0

//#define DRIVE_MAX 100
//#define DRIVE_MIN 10

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

#define ROLLING_AVG_DATA_POINTS 100
#define TEMP_AVERAGING_INITIAL_VALUE (float)21
class RollingAverage
{
public:
  RollingAverage() : val(), average(0) {}
  RollingAverage(float initialValue) : val(), average(initialValue)
  {
    for (uint8_t i = 0; i < ROLLING_AVG_DATA_POINTS; i++)
      val[i] = initialValue;
  }

  void push(float value)
  {
    for (uint8_t i = 1; i < ROLLING_AVG_DATA_POINTS; i++)
      val[i-1] = val[i];
    val[ROLLING_AVG_DATA_POINTS - 1] = value;
    updateAverage();
  }

  void set(float value)
  {
    for (uint8_t i = 1; i < ROLLING_AVG_DATA_POINTS; i++)
      val[i] = value;
    average = value;
  }

  float getAverage() { return average; }

  uint8_t getSize() { return ROLLING_AVG_DATA_POINTS; }

private:
  void updateAverage()
  {
    float sum = 0;
    for (uint8_t i = 0; i < ROLLING_AVG_DATA_POINTS; i++)
      sum += val[i];
    average = sum / (float)ROLLING_AVG_DATA_POINTS;
  }
  
  float val[ROLLING_AVG_DATA_POINTS];
  float average;
};

float tempRiseTarget, tempZero;
RollingAverage tempAvg(TEMP_AVERAGING_INITIAL_VALUE);
float heaterDutyCycle;
uint32_t timeZero, sampleTime, heaterOnTime, heaterPeriodTime, pidUpdateTime, printTime;
long sampleTimeLeft, heaterOnTimeLeft, heaterPeriodTimeLeft, pidUpdateTimeLeft, printTimeLeft;
SPid pid;
float pidDrive;
bool heaterIsOn;

float updatePID(SPid * pid, float err, float pos)
{
	float pTerm, dTerm, iTerm;

	pTerm = pid->pGain * err;
	
	pid->iState += err;
	if (pid->iState > pid->iMax) { pid->iState = pid->iMax; }
	if (pid->iState < pid->iMin) { pid->iState = pid->iMin; }
	iTerm = pid->iGain * pid->iState;

	dTerm = pid->dGain * (pid->dState - pos);
	pid->dState = pos;

	return pTerm + iTerm + dTerm;
}

#define TEMPERATURE_DATA_POINT_COUNT 10
float getTemperature()
{
  uint32_t sum = 0;
  for (uint8_t i = 0; i<TEMPERATURE_DATA_POINT_COUNT; i++)
    sum += analogRead(TEMP_ADC_PIN);
	return ( sum * 0.0048828125 / (float)TEMPERATURE_DATA_POINT_COUNT - 0.5 ) * 100.0 + 5.0;
}

float tempChangeToDuty(float tempChange)
{
  return min(1.0, max(0.0, tempChange * TEMP_TO_DUTY_SLOPE + TEMP_TO_DUTY_OFFSET));
}

float getErr()
{
  return -(tempAvg.getAverage() - tempZero - tempRiseTarget) / tempRiseTarget;
}

float getPos()
{
  return (tempAvg.getAverage() - tempZero) / tempRiseTarget;
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
	printCsv(tempAvg.getAverage());
	printCsv(tempZero + tempRiseTarget);
  printCsv(tempZero);
	printCsv(heaterDutyCycle);
	printCsv(pid->pGain);
	printCsv(pid->iGain);
	Serial.println(pid->dGain);
}

void printDataHeader()
{
	printCsv(F("time [sec]"));
	printCsv(F("temperature"));
	printCsv(F("target temperature"));
  printCsv(F("baseline"));
	printCsv(F("heater duty cycle"));
	printCsv(F("pGain"));
	printCsv(F("iGain"));
	Serial.println(F("dGain"));
}

void setup()
{
	Serial.begin(9600);
	Serial.println(F("temp_ctrl.ino compiled " __DATE__ " at " __TIME__));
	Serial.println();
	printDataHeader();

  pinMode(HEATER_PIN, OUTPUT);
  heaterOff();
  
  for (uint8_t i = 0; i < tempAvg.getSize(); i++)
    tempAvg.push(getTemperature());
  tempZero = tempAvg.getAverage();
  tempRiseTarget = TARGET;

	pid.init();
	pidDrive = updatePID(&pid,getErr(),getPos());
	heaterDutyCycle = DUTY_INIT;

	timeZero = millis();
	sampleTime = millis();			// sample (etc) immediately
  printTime = millis();
	heaterPeriodTime = millis();	// start new period immediately
}

#ifdef OPEN_LOOP_MODE
void loop()
{
	sampleTimeLeft = sampleTime - millis();
  printTimeLeft = printTime - millis();
	heaterOnTimeLeft = heaterOnTime - millis();
	heaterPeriodTimeLeft = heaterPeriodTime - millis();

	if (sampleTimeLeft < 0)
	{
		tempAvg.push(getTemperature());
		sampleTime = millis() + SAMPLE_DELAY_MS;
	}

  if (printTimeLeft < 0)
  {
    printData(&pid);
    printTime = millis() + PRINT_DELAY_MS;
  }

	if (heaterOnTimeLeft < 0 && heaterIsOn)
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

#ifdef PID_MODE
void loop()
{
  sampleTimeLeft = sampleTime - millis();
  printTimeLeft = printTime - millis();
  pidUpdateTimeLeft = pidUpdateTime - millis();
  heaterOnTimeLeft = heaterOnTime - millis();
  heaterPeriodTimeLeft = heaterPeriodTime - millis();

  if (sampleTimeLeft < 0)
  {
    tempAvg.push(getTemperature());
    sampleTime = millis() + SAMPLE_DELAY_MS;
  }

  if (printTimeLeft < 0)
  {
    printData(&pid);
    printTime = millis() + PRINT_DELAY_MS;
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
    pidDrive = updatePID(&pid, getErr(), getPos());
//    heaterDutyCycle = tempChangeToDuty(pidDrive * tempRiseTarget);
    heaterDutyCycle = constrain(pidDrive, 0, 1);
    pidUpdateTime = millis() + PID_UPDATE_PERIOD_MS;
    
    DEBUG2(F("pidDrive        = "), pidDrive);
    DEBUG2(F("heaterDutyCycle = "), heaterDutyCycle);
  }
}
#endif // PID_MODE
