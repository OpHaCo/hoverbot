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

#include <Arduino.h>
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
Hoverboard* Hoverboard::_instance = NULL;

/**************************************************************************
 * Public Methods Definitions
 **************************************************************************/
Hoverboard::Hoverboard(const Hoverboard::Config& arg_config):
  _hallSensor1(arg_config._motor1Conf),
  _hallSensor2(arg_config._motor2Conf),
  _u16_powerPin(arg_config._u16_powerPin),
  _timer(),
  _e_state(POWER_OFF)
{
  pinMode(_u16_powerPin, OUTPUT);
  powerOff(); 
  
  _instance = this;
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
   digitalWrite(_u16_powerPin, LOW);
  //TODO check status 
  if(_e_state != POWER_OFF || _e_state != POWERING_OFF)
  {
    _e_state = POWERING_OFF;
    digitalWrite(_u16_powerPin, HIGH);
    _timer.begin(Hoverboard::timerIt, Hoverboard::SHORT_PRESS_DUR_MS*1000); 
  }
  return NO_ERROR; 
}

Hoverboard::EHoverboardErr Hoverboard::stop(void)
{
  /** Send a frame indicating user is not on hoverboard */
  
}

/**************************************************************************
 * Private Methods Definitions
 **************************************************************************/
Hoverboard::EHoverboardState Hoverboard::getState(void)
{
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

