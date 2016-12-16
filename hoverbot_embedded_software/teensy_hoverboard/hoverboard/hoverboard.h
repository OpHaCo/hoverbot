/******************************************************************************
 * 
 * @file    hoverboard.h
 * @author  Rémi Pincent - INRIA
 * @date    30/11/2016
 *
 * @brief Hoverboard class. Gives control over your hoverboard.
 *  Code must call idleControl() method regularly (<50ms period). 
 *  Listener calls must be short < 10ms
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

#include "hoverboard_listener.h"

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
      IDLE, 
      COMMON_SPEED
    }EHoverboardState;
    

    /** Hoverboard configuration */
    class Config
    {
      public :
        uint16_t _u16_powerPin;
        BrushlessHallSensor::Config* _p_motor1Conf;
        BrushlessHallSensor::Config* _p_motor2Conf;
				HardwareSerial * _p_gyro1Serial, * _p_gyro2Serial;				

        inline Config(const uint16_t arg_u16_powerPin,
            BrushlessHallSensor::Config* arg_p_motor1HallSensorConf,
            BrushlessHallSensor::Config* arg_p_motor2HallSensorConf,
						HardwareSerial& arg_gyro1Serial,
						HardwareSerial& arg_gyro2Serial):
          _u16_powerPin(arg_u16_powerPin),
          _p_motor1Conf(arg_p_motor1HallSensorConf),
          _p_motor2Conf(arg_p_motor2HallSensorConf),
					_p_gyro1Serial(&arg_gyro1Serial),
					_p_gyro2Serial(&arg_gyro2Serial)
				{};
    };
    
  /** Private types */
  private :
    
    /** Events stored in bit field - max nb events = 32*/
    typedef enum {
      POWERED_ON_EVENT   =   1 << 0,
      POWERED_OFF_EVENT    =   1 << 1,
      SPEED_APPLIED_EVENT =   1 << 2
    }EEvent;

    /** private members */  
  private :
    /** Optional - motor 1,2 hall sensors */
    bool _has_motor1HallSensor,  _has_motor2HallSensor;
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
    float _f_sensorSpeed1, _f_sensorSpeed2; 
    float _f_diffSpeed1, _f_diffSpeed2; 
    int16_t _s16_speedRampUp; 
    float _f_commonSpeed;
    /** Factor to reach target speed - time to reach speed = 1/ramp_up_factor */ 
    float _f_rampUpFactor;
    /** Pending events pushed under IT : bit field - max events = 32 */
    uint32_t _u32_events; 
    HoverboardListener* _p_listener; 
    
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
    /** 1 single listener */
    void registerListener(HoverboardListener*);
    void unregisterListener(void);
    EHoverboardErr powerOn(void);
    EHoverboardErr powerOnAsync(void);
    EHoverboardErr powerOff(void);
    EHoverboardErr powerOffAsync(void);

    /** Set speed for motors  */ 
    EHoverboardErr setSpeedAsync(float arg_f_speed1, float arg_f_speed2);
    
    /**
     * Stop any control 
     **/
    EHoverboardErr stop(void); 
    
    /** This function must always called in main loop.
     * If it is not called during some 100ms. Hoverboard
     * will enter error mode */
    void idleControl(void); 

		/** TODO get target range  - make it private*/
    void simulateGyro1(int16_t arg_s16_target);
    void simulateGyro2(int16_t arg_s16_target);

    /** private methods */  
  private :
    EHoverboardState getState(void);
    void setCommonSpeedAsync(float arg_f_speed1, float arg_f_speed2);
    void setDifferentialSpeed(float arg_f_speed1, float arg_f_speed2);
    static void timerIt(void);
    
    /** Motor 1 sensor Cbs */
    static void motor1Hall1It(void);
    static void motor1Hall2It(void);
    static void motor1Hall3It(void);
    static void motor1TimerIt(void);
    
    /** Motor 2 sensor Cbs */
    static void motor2Hall1It(void);
    static void motor2Hall2It(void);
    static void motor2Hall3It(void);
    static void motor2TimerIt(void);
};

#endif /* HOVERBOARD_H */
