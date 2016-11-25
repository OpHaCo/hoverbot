/******************************************************************************
 * @file    brushless_hall_sensor.cpp
 * @author  Rémi Pincent - INRIA
 * @date    24/11/2016
 *
 * @brief Get sensor speed in deg/s from some hall sensors
 *
 * Project : hoverbot
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 *
 * Revision History:
 * https://github.com/OpHaCo/hoverbot.git 
 * 
 * LICENSE :
 * hoverbot (c) by Rémi Pincent
 * hoverbot is licensed under a
 * Creative Commons Attribution-NonCommercial 3.0 Unported License.
 *
 * You should have received a copy of the license along with this
 * work.  If not, see <http://creativecommons.org/licenses/by-nc/3.0/>.
 *****************************************************************************/

/**************************************************************************
 * Include Files
 **************************************************************************/
#include "brushless_hall_sensor.h"

#include <Arduino.h>

/**************************************************************************
 * Manifest Constants
 **************************************************************************/
#define PI 3.141592
/**************************************************************************
 * Type Definitions
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Static Variables
 **************************************************************************/
BrushlessHallSensor* BrushlessHallSensor::_instance = NULL;
/** To customize depending on your hall sensors */
const float BrushlessHallSensor::NB_TICKS_ROTA = 44.7;
const uint16_t BrushlessHallSensor::TIMER_PERIOD_MS = 2500;

/**************************************************************************
 * Macros
 **************************************************************************/

/**************************************************************************
 * Local Functions Declarations
 **************************************************************************/

/**************************************************************************
 * Public Functions Defintions
 **************************************************************************/
BrushlessHallSensor::BrushlessHallSensor(uint8_t arg_u8_hall1Pin, uint8_t arg_u8_hall2Pin, uint8_t arg_u8_hall3Pin):
  _s32_hallTicks(0),
  _s32_periodHallTicks(0),
  _s32_lastPeriodHallTicks(0),
  _e_lastHallSensed(BrushlessHallSensor::OUT_OF_ENUM_SENSOR),
  _u8_hall1Pin(arg_u8_hall1Pin),
  _u8_hall2Pin(arg_u8_hall2Pin),
  _u8_hall3Pin(arg_u8_hall3Pin),
  _captureTimer()
{
  /** quick singleton ! */
  BrushlessHallSensor::_instance = this;
}

void BrushlessHallSensor::startSensing(void)
{
  BrushlessHallSensor::_s32_hallTicks = 0;
  BrushlessHallSensor::_e_lastHallSensed = BrushlessHallSensor::OUT_OF_ENUM_SENSOR;

  pinMode(_u8_hall1Pin, INPUT_PULLUP);
  pinMode(_u8_hall2Pin, INPUT_PULLUP);
  pinMode(_u8_hall3Pin, INPUT_PULLUP);
  
  attachInterrupt(_u8_hall1Pin, BrushlessHallSensor::hall1It, RISING); 
  attachInterrupt(_u8_hall2Pin, BrushlessHallSensor::hall2It, RISING); 
  attachInterrupt(_u8_hall3Pin, BrushlessHallSensor::hall3It, RISING); 
  
  _captureTimer.begin(BrushlessHallSensor::timerIt, BrushlessHallSensor::TIMER_PERIOD_MS*1000);
  _s32_periodHallTicks = 0;
  _s32_lastPeriodHallTicks = 0;
}

void BrushlessHallSensor::stopSensing(void)
{
  _captureTimer.end();
  detachInterrupt(_u8_hall1Pin);
  detachInterrupt(_u8_hall2Pin);
  detachInterrupt(_u8_hall3Pin);
}

float BrushlessHallSensor::getSpeed(void)
{
  /** No need to do a copy to avoid a concurrent write, _s32_lastPeriodHallTicks is a world */
  return PI*(((float)_s32_lastPeriodHallTicks*1000)/(float)TIMER_PERIOD_MS)/NB_TICKS_ROTA;
}
/**************************************************************************
 * Private Functions Definitions
 **************************************************************************/
void BrushlessHallSensor::hall1It(void)
{
  detachInterrupt(_instance->_u8_hall1Pin);

  /** reject spikes */
  if(digitalRead(_instance->_u8_hall1Pin) &&
      ((digitalRead(_instance->_u8_hall2Pin) && !digitalRead(_instance->_u8_hall3Pin))
       || (!digitalRead(_instance->_u8_hall2Pin) && digitalRead(_instance->_u8_hall3Pin))))
  {

    if(_instance->_e_lastHallSensed == BrushlessHallSensor::SENSOR_3)
    {
      _instance->_s32_hallTicks++;
    }
    else if(_instance->_e_lastHallSensed == BrushlessHallSensor::SENSOR_2)
    {
      _instance->_s32_hallTicks--;
    }
    _instance->_e_lastHallSensed = BrushlessHallSensor::SENSOR_1;
  }
  attachInterrupt(_instance->_u8_hall1Pin, BrushlessHallSensor::hall1It, RISING); 
}

void BrushlessHallSensor::hall2It(void)
{
  detachInterrupt(_instance->_u8_hall2Pin);

  /** reject spikes */
  if(digitalRead(_instance->_u8_hall2Pin) &&
      ((digitalRead(_instance->_u8_hall1Pin) && !digitalRead(_instance->_u8_hall3Pin))
       || (!digitalRead(_instance->_u8_hall1Pin) && digitalRead(_instance->_u8_hall3Pin))))
  {
    if(_instance->_e_lastHallSensed == BrushlessHallSensor::SENSOR_1)
    {
      _instance->_s32_hallTicks++;
    }
    else if(_instance->_e_lastHallSensed == BrushlessHallSensor::SENSOR_3)
    {
      _instance->_s32_hallTicks--;
    }
    _instance->_e_lastHallSensed = BrushlessHallSensor::SENSOR_2;
  }
  attachInterrupt(_instance->_u8_hall2Pin, BrushlessHallSensor::hall2It, RISING); 
}

void BrushlessHallSensor::hall3It(void)
{
  detachInterrupt(_instance->_u8_hall3Pin);

  /** reject spikes */
  if(digitalRead(_instance->_u8_hall3Pin) &&
      ((digitalRead(_instance->_u8_hall1Pin) && !digitalRead(_instance->_u8_hall2Pin))
       || (!digitalRead(_instance->_u8_hall1Pin) && digitalRead(_instance->_u8_hall2Pin))))
  {
    if(_instance->_e_lastHallSensed == BrushlessHallSensor::SENSOR_2)
    {
      _instance->_s32_hallTicks++;
    }
    else if(_instance->_e_lastHallSensed == BrushlessHallSensor::SENSOR_1)
    {
      _instance->_s32_hallTicks--;
    }
    _instance->_e_lastHallSensed = BrushlessHallSensor::SENSOR_3;
  }
  attachInterrupt(_instance->_u8_hall3Pin, BrushlessHallSensor::hall3It, RISING); 
}


void BrushlessHallSensor::timerIt(void){
  _instance->_s32_lastPeriodHallTicks = _instance->_s32_hallTicks - _instance->_s32_periodHallTicks;
  _instance->_s32_periodHallTicks = _instance->_s32_hallTicks;
}
