/******************************************************************************
 * @file    brushless_hall_sensor.h
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
#ifndef BRUSHLESS_HALL_SENSOR
#define BRUSHLESS_HALL_SENSOR

/**************************************************************************
 * Include Files
 **************************************************************************/
#include <stdint.h>
#include <IntervalTimer.h>
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
class BrushlessHallSensor
{
  /** Public types */
  public :
    typedef enum{
      SENSOR_1,
      SENSOR_2,
      SENSOR_3,
      OUT_OF_ENUM_SENSOR
    }EHallSensor;

    typedef void(*ITCb)(void);
    
    class ItCbs
    {
      public :
        ITCb _pfn_hall1It, _pfn_hall2It, _pfn_hall3It, _pfn_timerIt; 
        inline ItCbs(void){} 
        inline ItCbs(ITCb arg_pfn_hall1It,
            ITCb arg_pfn_hall2It, 
            ITCb arg_pfn_hall3It,
            ITCb arg_pfn_timerIt):
              _pfn_hall1It(arg_pfn_hall1It),
              _pfn_hall2It(arg_pfn_hall2It),
              _pfn_hall3It(arg_pfn_hall3It),
              _pfn_timerIt(arg_pfn_timerIt)
      {}
    };

    class Config
    {
      public :
        uint8_t _u8_hall1Pin, _u8_hall2Pin, _u8_hall3Pin;
        ItCbs _itCbs;
        inline Config(void){}
        inline Config(const uint8_t arg_u8_hall1Pin, 
            const uint8_t arg_u8_hall2Pin, 
            const uint8_t arg_u8_hall3Pin, 
            ItCbs& arg_itCbs):
          _u8_hall1Pin(arg_u8_hall1Pin),
          _u8_hall2Pin(arg_u8_hall2Pin),
          _u8_hall3Pin(arg_u8_hall3Pin)
      {
        _itCbs = arg_itCbs;
      }
    };
    
    
    /** public members */  
  public :
    /** Number of ticks for a 2pi rotation */
    static const float NB_TICKS_ROTA;

    /** private members */  
  private :
    Config _config;
    volatile int32_t _s32_hallTicks, _s32_periodHallTicks, _s32_lastPeriodHallTicks;
    volatile EHallSensor _e_lastHallSensed;

    /** Timer used to capture speed */
    IntervalTimer _captureTimer;
    /** Timer capture period */
    static const uint16_t TIMER_PERIOD_MS;

    /** public methods */
  public :
    BrushlessHallSensor(void);
    BrushlessHallSensor(const Config&); 

    void startSensing(void);
    void stopSensing(void);
    /**
     * Get speed in rad/s 
     */
    float getSpeed(void); 
    inline int32_t getTicks(void){return _s32_hallTicks;}
  
    /**
     * These methods must be called when callback registered in 
     * config are called */
    void hall1It(void);
    void hall2It(void);
    void hall3It(void);
    void timerIt(void);
    /** private methods */  
  private :
};

#endif /* BRUSHLESS_HALL_SENSOR */
