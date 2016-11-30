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
#include <Arduino.h>
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
				HardwareSerial * _p_gyro1Serial, * _p_gyro2Serial;				

        inline Config(const uint16_t arg_u16_powerPin,
            BrushlessHallSensor::Config& arg_motor1HallSensorConf,
            BrushlessHallSensor::Config& arg_motor2HallSensorConf,
						HardwareSerial& arg_gyro1Serial,
						HardwareSerial& arg_gyro2Serial):
          _u16_powerPin(arg_u16_powerPin),
          _motor1Conf(arg_motor1HallSensorConf),
          _motor2Conf(arg_motor2HallSensorConf),
					_p_gyro1Serial(&arg_gyro1Serial),
					_p_gyro2Serial(&arg_gyro2Serial)
				{};
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
    int16_t _s16_calValue;
		HardwareSerial * _p_gyro1Serial, * _p_gyro2Serial;				
    float _f_speed1, _f_speed2;
    /** Factor to reach target speed - time to reach speed = 1/ramp_up_factor */ 
    float _f_rampUpFactor;
    
    static Hoverboard* _instance; 
    
    static const uint8_t SHORT_PRESS_DUR_MS;
		/** Gyro daughterboard frame length : depends on hoverboard firmware */
		static const uint8_t GYRO_FRAME_LENGTH;
		/** Byte sent in gyro frame indicating user has its feet on gyro board */
		static const uint8_t GYRO_CONTACT_CLOSED_BYTE;
		/** Byte sent in gyro frame indicating user does not have its feet on gyro board */
		static const uint8_t GYRO_CONTACT_OPENED_BYTE;
		static const uint16_t GYRO_FRAME_START;

    /** public methods */
  public :
    Hoverboard(const Hoverboard::Config&);
		EHoverboardErr init(void);
    EHoverboardErr powerOn(void);
    EHoverboardErr powerOnAsync(void);
    EHoverboardErr powerOff(void);
    EHoverboardErr powerOffAsync(void);

    /** Set speed for motors  */ 
    EHoverboardErr setSpeed(float arg_f_speed1, float arg_f_speed2);
    
    /**
     * Stop any control 
     **/
    EHoverboardErr stop(void); 
    
    /** This function must always called in main loop.
     * If it is not called during some 100ms. Hoverboard
     * will enter error mode */
    void idleControl(void); 

    EHoverboardErr calibrate(void);

		/** TODO get target range  - make it private*/
    void simulateGyro1(int16_t arg_s16_target);
    void simulateGyro2(int16_t arg_s16_target);

    /** private methods */  
  private :
    EHoverboardState getState(void);
    void setSpeedSameRota(float arg_f_speed1, float arg_f_speed2);
    static void timerIt(void);
};

#endif /* HOVERBOARD_H */
