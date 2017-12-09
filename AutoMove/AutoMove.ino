/* This file is used (abused) to put some of the tertiary and lower library
 * #includes into the forefront. It doesn't seem to work otherwise, when a 
 * library #includes another library. */

#include "RollingAverage.h"
#include "AutoMoveSD.h"
#include <Arduino.h>
#include <inttypes.h>
#include <limits.h>
#include <SysCall.h>
#include <SdFatConfig.h>
#include <SdFat.h>
#include <MinimumSerial.h>
#include <FreeStack.h>
#include <BlockDriver.h>
#include <SPI.h>
#include "TopoScan.h"
#include "SonarSensor.h"
#include <Servo.h>
#include "ForceSensor.h"

void setup() {}

void loop() {}
