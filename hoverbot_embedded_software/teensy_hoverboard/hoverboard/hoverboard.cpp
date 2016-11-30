/******************************************************************************
 * @file    hoverboard.cpp
 * @author  Rémi Pincent - INRIA
 * @date    28/11/2016
 *
 * @brief Hoverboard implementation. Hoverboard control done hacking : 
 *  - gyroscope daughterboards signals to control speed
 *  - power button
 *  - Brushless motors Hall sensors (3 per motors)
 *  - Status LED to get ERROR signal
 *  
 * Requires : 
 *  - 1 UART to simulate daughterboard gyroscope 1
 *  - 1 UART to simulater daughterboard gyroscope 2
 *  - 3x2 digital INPUTS to get Hall sensors pulses
 *  - 1 digital OUTPUT to simulate power button press
 *  - 1 digital INPUT to get ERROR status
 *  
 * Project : hoverbot
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 *
 * Revision History:
 * https://github.com/OpHaCo/hoverbot.git
 * 
 * LICENSE :
 * hoverbot (c) by Rémi Pincent
 * project_name is licensed under a
 * Creative Commons Attribution-NonCommercial 3.0 Unported License.
 *
 * You should have received a copy of the license along with this
 * work.  If not, see <http://creativecommons.org/licenses/by-nc/3.0/>.
 *****************************************************************************/

/**************************************************************************
 * Include Files
 **************************************************************************/
#include "hoverboard.h"

#include <IntervalTimer.h>

/**************************************************************************
 * Manifest Constants
 **************************************************************************/

/**************************************************************************
 * Type Definitions
 **************************************************************************/

/**************************************************************************
 * Static members definitions
 **************************************************************************/
const uint8_t Hoverboard::SHORT_PRESS_DUR_MS = 100;
const uint8_t Hoverboard::GYRO_FRAME_LENGTH = 6;
const uint8_t Hoverboard::GYRO_CONTACT_CLOSED_BYTE = 85;
const uint8_t Hoverboard::GYRO_CONTACT_OPENED_BYTE = 170;
const uint16_t Hoverboard::GYRO_FRAME_START = 256;

Hoverboard* Hoverboard::_instance = NULL;

/**************************************************************************
 * Public Methods Definitions
 **************************************************************************/
Hoverboard::Hoverboard(const Hoverboard::Config& arg_config):
  _hallSensor1(arg_config._motor1Conf),
  _hallSensor2(arg_config._motor2Conf),
  _u16_powerPin(arg_config._u16_powerPin),
  _timer(),
  _e_state(POWER_OFF),
	_s16_calValue(0),
	_p_gyro1Serial(arg_config._p_gyro1Serial),
	_p_gyro2Serial (arg_config._p_gyro2Serial),
  _f_speed1(0.0),
  _f_speed2(0.0),
  _f_rampUpFactor(0.3)
{
  _instance = this;
}  
  
Hoverboard::EHoverboardErr Hoverboard::init(void)
{
	 _p_gyro1Serial->begin (26300, SERIAL_9N1);  // 9 bits mode
   _p_gyro2Serial->begin (26300, SERIAL_9N1);  // 9 bits mode

  pinMode(_u16_powerPin, OUTPUT);
  powerOff(); 

	return NO_ERROR;
}
  
Hoverboard::EHoverboardErr Hoverboard::powerOn(void)
{
  powerOnAsync();
  while(_e_state != POWER_ON)
  {
    /** TODO : IDLE control */
  }
  //TODO check status 
  return NO_ERROR;
} 
  
Hoverboard::EHoverboardErr Hoverboard::powerOnAsync(void)
{
  if(_e_state != POWER_ON || _e_state != POWERING_ON)
  {
    _e_state = POWERING_ON;
    digitalWrite(_u16_powerPin, HIGH);
    _timer.begin(Hoverboard::timerIt, Hoverboard::SHORT_PRESS_DUR_MS*1000); 
  }
  return NO_ERROR; 
} 

Hoverboard::EHoverboardErr Hoverboard::powerOff(void)
{
  powerOffAsync();
  while(_e_state != POWER_OFF)
  {
    /** TODO : IDLE control */
  }
  //TODO check status 
  return NO_ERROR;
}

Hoverboard::EHoverboardErr Hoverboard::powerOffAsync(void)
{
  //TODO check status 
  if(_e_state != POWER_OFF || _e_state != POWERING_OFF)
  {
		Serial.println("powering off");
    _e_state = POWERING_OFF;
    digitalWrite(_u16_powerPin, HIGH);
    _timer.begin(Hoverboard::timerIt, Hoverboard::SHORT_PRESS_DUR_MS*1000); 
  }
  return NO_ERROR; 
}

Hoverboard::EHoverboardErr Hoverboard::setSpeed(float arg_f_speed1, float arg_f_speed2)
{
  /** Are both motors rotating in same direction ? */
  if((arg_f_speed1 >= 0 && arg_f_speed2 >= 0) || (arg_f_speed1 < 0 && arg_f_speed2 < 0)) 
  {
    setSpeedSameRota(arg_f_speed1, arg_f_speed2); 
  } 
  else /** Motors do not rotate in same direction */
  {
  }
  _f_speed1 = arg_f_speed1; 
  _f_speed2 = arg_f_speed2; 
} 
    
Hoverboard::EHoverboardErr Hoverboard::stop(void)
{
}

void Hoverboard::idleControl(void)
{
  /** Idle control => apply calibration value when motors rotate same direction */
  simulateGyro1(_s16_calValue);
  simulateGyro2(-_s16_calValue);
}
/**************************************************************************
 * Private Methods Definitions
 **************************************************************************/
Hoverboard::EHoverboardState Hoverboard::getState(void)
{
}

void Hoverboard::simulateGyro1(int16_t arg_s16_target)
{
	/** send a frame simulating gyroscope move on gyro 
	 * daughterboard 1 */
	uint16_t loc_au16_gyroFrame[GYRO_FRAME_LENGTH] = {
		GYRO_FRAME_START, 
		(uint8_t)(arg_s16_target & 0xff), 
		(uint8_t)(arg_s16_target >> 8 & 0xff), 
		(uint8_t)(arg_s16_target & 0xff), 
		(uint8_t)(arg_s16_target >> 8 & 0xff), 
		GYRO_CONTACT_CLOSED_BYTE};

	for(uint8_t index = 0; index < 6; index++)
	{
		_p_gyro1Serial->write9bit(loc_au16_gyroFrame[index]); 
	} 
}  
    
void Hoverboard::simulateGyro2(int16_t arg_s16_target)
{
  /** send a frame simulating gyroscope move on gyro 
   * daughterboard 2 */
  uint16_t loc_au16_gyroFrame[GYRO_FRAME_LENGTH] = {
    GYRO_FRAME_START, 
    (uint8_t)(arg_s16_target & 0xff), 
    (uint8_t)(arg_s16_target >> 8 & 0xff), 
    (uint8_t)(arg_s16_target & 0xff), 
    (uint8_t)(arg_s16_target >> 8 & 0xff), 
    GYRO_CONTACT_CLOSED_BYTE};
  
  for(uint8_t index = 0; index < 6; index++)
  {
    _p_gyro2Serial->write9bit(loc_au16_gyroFrame[index]);
  }
}

/**
* When motors rotate in same direction. Hoverboard performs an integral
* control from daugtherboards gyroscope data. In order to keep a clean 
* control, we must simulate an integral control on simulated gyros/
*/ 
void Hoverboard::setSpeedSameRota(float arg_f_speed1, float arg_f_speed2)
{
  int16_t  loc_s16_speedRampUp1 = (arg_f_speed1 > 0) ? (arg_f_speed1 - _f_speed1)*_f_rampUpFactor : -(arg_f_speed1 - _f_speed1)*_f_rampUpFactor;      
  int16_t  loc_s16_speedRampUp2 = (arg_f_speed2 > 0) ? (arg_f_speed2 - _f_speed2)*_f_rampUpFactor : -(arg_f_speed2 - _f_speed2)*_f_rampUpFactor;      

  Serial.println(arg_f_speed1);
  Serial.println(arg_f_speed2);
  Serial.println(loc_s16_speedRampUp1);
  Serial.println(loc_s16_speedRampUp2);
  uint32_t loc_u32_timeStart = millis();
  while((millis() - loc_u32_timeStart) < (1/_f_rampUpFactor)*1000)
  {
    /** Same rota direction => refer README.md - Gyro simualated target must from different
     * signum */
    simulateGyro1(loc_s16_speedRampUp1);
    simulateGyro2(-loc_s16_speedRampUp2);
  }
} 

void Hoverboard::timerIt(void)
{
  if(_instance->_e_state == POWERING_ON)
  {
    digitalWrite(_instance->_u16_powerPin, LOW);
    _instance->_e_state = POWER_ON;
  }
  else if(_instance->_e_state == POWERING_OFF)
  {
    digitalWrite(_instance->_u16_powerPin, LOW);
    _instance->_e_state = POWER_OFF;
  }
  _instance->_timer.end();
}


