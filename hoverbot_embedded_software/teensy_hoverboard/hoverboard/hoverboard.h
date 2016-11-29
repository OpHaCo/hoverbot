/******************************************************************************
 * 
 * @file    hoverboard_hack.h
 * @author  Rémi Pincent - INRIA
 * @date    30/11/2016
 *
 * @brief Hoverboard class. Gives control over your hoverboard.
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
#ifndef HOVERBOARD_H
#define HOVERBOARD_H

/**************************************************************************
 * Include Files
 **************************************************************************/
#include <stdint.h>
#include <brushless_hall_sensor.h>

/**************************************************************************
 * Manifest Constants
 **************************************************************************/

/**************************************************************************
 * Type Definitions
 **************************************************************************/

/**************************************************************************
 * Global variables
 **************************************************************************/

/**************************************************************************
 * Macros
 **************************************************************************/

/**************************************************************************
 *  Class Declarations
 **************************************************************************/
class Hoverboard 
{
  /** Public types */
  public :
    typedef enum {
      NO_ERROR = 0,
    }EHoverboardErr;
    
    typedef enum {
      POWER_OFF,
      CALIBRATING,
      POWER_ON,
      POWERING_ON,
      POWERING_OFF,
    }EHoverboardState;

    /** Hoverboard configuration */
    class Config
    {
      public :
        uint16_t _u16_powerPin;
        BrushlessHallSensor::Config _motor1Conf;
        BrushlessHallSensor::Config _motor2Conf;
        inline Config(const uint16_t arg_u16_powerPin,
            BrushlessHallSensor::Config& arg_motor1HallSensorConf,
            BrushlessHallSensor::Config& arg_motor2HallSensorConf):
          _u16_powerPin(arg_u16_powerPin),
          _motor1Conf(arg_motor1HallSensorConf),
          _motor2Conf(arg_motor2HallSensorConf){};
    };

    /** private members */  
  private :
    /** Motor 1,2 hall sensors */
    BrushlessHallSensor _hallSensor1, _hallSensor2;
    /** Pin to control hoverboard power, calibration */
    const uint16_t _u16_powerPin; 
    /** Hoverboard must always get some control commands
     * => no delay fonction > 50ms must be called, or an error 
     * on hoverboard side will be detected.
     * Timer is used to handle delay asynchronously 
     **/
    IntervalTimer _timer;
    volatile EHoverboardState _e_state;
    static Hoverboard* _instance; 
    
    static const uint8_t SHORT_PRESS_DUR_MS;

    /** public methods */
  public :
    Hoverboard(const Hoverboard::Config&);
    EHoverboardErr powerOn(void);
    EHoverboardErr powerOnAsync(void);
    EHoverboardErr powerOff(void);
    EHoverboardErr powerOffAsync(void);
    /**
     * Stop any control 
     **/
    EHoverboardErr stop(void); 
    
    /** This function must always called in main loop.
     * If it is not called during some 100ms. Hoverboard
     * will enter error mode */
    void idleControl(void); 

    /** private methods */  
  private :
    EHoverboardState getState(void);
    static void timerIt(void);
};

#endif /* HOVERBOARD_H */
