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
/**************************************************************************
 * Type Definitions
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Static Variables
 **************************************************************************/
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

BrushlessHallSensor::BrushlessHallSensor(void) :
  _config(),
  _s32_hallTicks(0),
  _s32_periodHallTicks(0),
  _s32_lastPeriodHallTicks(0),
  _e_lastHallSensed(BrushlessHallSensor::OUT_OF_ENUM_SENSOR),
  _captureTimer()
{
}

BrushlessHallSensor::BrushlessHallSensor(const Config& arg_config):BrushlessHallSensor()
{
  _config = arg_config;
}

void BrushlessHallSensor::startSensing(void)
{
  BrushlessHallSensor::_s32_hallTicks = 0;
  BrushlessHallSensor::_e_lastHallSensed = BrushlessHallSensor::OUT_OF_ENUM_SENSOR;

  pinMode(_config._u8_hall1Pin, INPUT_PULLUP);
  pinMode(_config._u8_hall2Pin, INPUT_PULLUP);
  pinMode(_config._u8_hall3Pin, INPUT_PULLUP);
  
  
  attachInterrupt(_config._u8_hall1Pin, _config._itCbs._pfn_hall1It, RISING); 
  attachInterrupt(_config._u8_hall2Pin, _config._itCbs._pfn_hall2It, RISING); 
  attachInterrupt(_config._u8_hall3Pin, _config._itCbs._pfn_hall3It, RISING); 
  
  _captureTimer.begin(_config._itCbs._pfn_timerIt, BrushlessHallSensor::TIMER_PERIOD_MS*1000);
  _s32_periodHallTicks = 0;
  _s32_lastPeriodHallTicks = 0;
}

void BrushlessHallSensor::stopSensing(void)
{
  _captureTimer.end();
  detachInterrupt(_config._u8_hall1Pin);
  detachInterrupt(_config._u8_hall2Pin);
  detachInterrupt(_config._u8_hall3Pin);
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
  detachInterrupt(_config._u8_hall1Pin); 
  /** reject spikes */
  if(digitalRead(_config._u8_hall1Pin) &&
      ((digitalRead(_config._u8_hall2Pin) && !digitalRead(_config._u8_hall3Pin))
       || (!digitalRead(_config._u8_hall2Pin) && digitalRead(_config._u8_hall3Pin))))
  {

    if(_e_lastHallSensed == BrushlessHallSensor::SENSOR_3)
    {
      _s32_hallTicks++;
    }
    else if(_e_lastHallSensed == BrushlessHallSensor::SENSOR_2)
    {
      _s32_hallTicks--;
    }
    _e_lastHallSensed = BrushlessHallSensor::SENSOR_1;
  }
  attachInterrupt(_config._u8_hall1Pin, _config._itCbs._pfn_hall1It, RISING); 
}

void BrushlessHallSensor::hall2It(void)
{
  detachInterrupt(_config._u8_hall2Pin); 
  /** reject spikes */
  if(digitalRead(_config._u8_hall2Pin) &&
      ((digitalRead(_config._u8_hall1Pin) && !digitalRead(_config._u8_hall3Pin))
       || (!digitalRead(_config._u8_hall1Pin) && digitalRead(_config._u8_hall3Pin))))
  {
    if(_e_lastHallSensed == BrushlessHallSensor::SENSOR_1)
    {
      _s32_hallTicks++;
    }
    else if(_e_lastHallSensed == BrushlessHallSensor::SENSOR_3)
    {
      _s32_hallTicks--;
    }
    _e_lastHallSensed = BrushlessHallSensor::SENSOR_2;
  }
  attachInterrupt(_config._u8_hall2Pin, _config._itCbs._pfn_hall2It, RISING); 
}

void BrushlessHallSensor::hall3It(void)
{
  detachInterrupt(_config._u8_hall3Pin); 
  /** reject spikes */
  if(digitalRead(_config._u8_hall3Pin) &&
      ((digitalRead(_config._u8_hall1Pin) && !digitalRead(_config._u8_hall2Pin))
       || (!digitalRead(_config._u8_hall1Pin) && digitalRead(_config._u8_hall2Pin))))
  {
    if(_e_lastHallSensed == BrushlessHallSensor::SENSOR_2)
    {
      _s32_hallTicks++;
    }
    else if(_e_lastHallSensed == BrushlessHallSensor::SENSOR_1)
    {
      _s32_hallTicks--;
    }
    _e_lastHallSensed = BrushlessHallSensor::SENSOR_3;
  }
  attachInterrupt(_config._u8_hall3Pin, _config._itCbs._pfn_hall3It, RISING); 
}


void BrushlessHallSensor::timerIt(void){
  _s32_lastPeriodHallTicks = _s32_hallTicks - _s32_periodHallTicks;
  _s32_periodHallTicks = _s32_hallTicks;
}
