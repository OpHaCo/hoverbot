/******************************************************************************
 * @file    hoverboard sketch
 * @author  Rémi Pincent - INRIA
 * @date    28/11/2016
 *
 * @brief Hoverboard sketch example to control hoverboard.
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
#include <logger_config.h>
#include <hoverboard.h>
#include <brushless_hall_sensor.h>

/**************************************************************************
 * Manifest Constants
 **************************************************************************/
#define LOG_SPEED 500000
#define LOG_SERIAL Serial
#define MAX_CMD_LENGTH 0xFFU

#define SET_SPEED_CMD_LENGTH 13U 
#define POWER_ON_CMD_LENGTH   1U 
#define POWER_OFF_CMD_LENGTH  1U 
#define STOP_CMD_LENGTH       1U 
/**************************************************************************
 * Type Definitions
 **************************************************************************/
/*****************************************
 * CONTROL cmd format
 *  __________________________________
 * |       |             |            |
 * |CMD_ID | CMD_PARAMS  | TODO CRC   |
 * |_______|_____________|____________|
 * 
 ****************************************/
typedef enum
{
  SET_SPEED    = 0, /* ARGS : SPEED1 (short LE) - SPEED2 (short LE) */
  POWER_ON     = 1,
  POWER_OFF    = 2,
  STOP         = 3
}EControlCmd;

typedef enum
{
  ACK          = 0,
  SPEED_STATUS = 1
}EFeedbackCmds;

typedef enum
{
  ACK_OK        = 0, 
  ACK_BAD_STATE = 1
}ECmdAckType;

/**
 * Class that listen hoverboard commands over UART to control it.
 */
class HoverboardListenerExample : public HoverboardListener
{
  public:
    HoverboardListenerExample(Hoverboard* );
    void init(void);
    void calibrateHoverboard(void);
    void onSpeedChanged(float arg_f_motor1Speed, float arg_f_motor2Speed);
    void onSpeedApplied(void);
    void onPowerOn(void);
    void onPowerOff(void);
    void pollCmd(void);
    void sendCmd(EFeedbackCmds arg_e_cmdType, uint8_t arg_au8_cmd[], uint8_t arg_u8_cmdLength);
    
  private:
    void waitKey(uint8_t arg_u8_key);
    void handleCommand(uint8_t arg_u8_cmd[], uint16_t arg_u16_cmdLength);

  private:
    Hoverboard* _p_hoverboard;
};

/**************************************************************************
 * Static variables definitions
 **************************************************************************/
static uint8_t _au8_rcvCmd[MAX_CMD_LENGTH] = {0};

/** ItCBS will be filled by hoverboard */
static BrushlessHallSensor::ItCbs motor1ItCbs;

/** Hall sensor 1 - Give pin here */
static BrushlessHallSensor::Config _hallSensor1Conf(17, 18, 19, motor1ItCbs);

/** ItCBS will be filled by hoverboard */
static BrushlessHallSensor::ItCbs motor2ItCbs;

/** Hall sensor 2 - Give pin here */
static BrushlessHallSensor::Config _hallSensor2Conf(13, 14, 15, motor2ItCbs);

static Hoverboard::Config _hoverboardConf(20, 
  21,
  &_hallSensor1Conf,
  &_hallSensor2Conf,
  Serial1,
  Serial3);

static Hoverboard _hoverboard(_hoverboardConf);
static HoverboardListenerExample _hoverboardListenerExample(&_hoverboard);
/**************************************************************************
 * Global functions Definitions
 **************************************************************************/
void setup() {
   LOG_SERIAL.begin(500000);
   LOG_INIT_STREAM(LOG_LEVEL, &LOG_SERIAL);
   /** Wait 1s to get setup logs */
   delay(2000);
   LOG_INFO_LN("starting hoverboard hack...");
   _hoverboard.init();
   _hoverboardListenerExample.init();
}

void loop() {
  _hoverboard.idleControl();
  
  _hoverboardListenerExample.pollCmd();
}

/**************************************************************************
 * HoverboardListenerExample Definitions
 **************************************************************************/

HoverboardListenerExample::HoverboardListenerExample(Hoverboard* arg_p_hoverboardExample)
{
  _p_hoverboard = arg_p_hoverboardExample;
}

void HoverboardListenerExample::init(void)
{
  _p_hoverboard->registerListener(this);
}

void HoverboardListenerExample::onSpeedChanged(float arg_f_motor1Speed, float arg_f_motor2Speed)
{
  LOG_INFO_LN("New speed (%f, %f)", arg_f_motor1Speed, arg_f_motor2Speed);   
}

void HoverboardListenerExample::onSpeedApplied(void)
{
  LOG_INFO_LN("SPEED_APPLIED"); 
}

void HoverboardListenerExample::onPowerOn(void)
{
  LOG_INFO_LN("POWERED_ON"); 
}

void HoverboardListenerExample::onPowerOff(void)
{
  LOG_INFO_LN("POWERED_OFF"); 
}

void HoverboardListenerExample::waitKey(uint8_t arg_u8_key)
{
  while(1) 
  {
    _p_hoverboard->idleControl();
    if(LOG_SERIAL.available() && LOG_SERIAL.read() == arg_u8_key)break;
  }
}

/**
 * Manual calibration : user must refer instructions on UART
 */
void HoverboardListenerExample::calibrateHoverboard(void)
{
  LOG_INFO_LN("Hoverboard calibration");
  LOG_INFO_LN("Power off hoverboard - press enter when done");
  waitKey('\r');

  LOG_INFO_LN("Power on hoverboard - press enter when done");
  waitKey('\r');
  
  LOG_INFO_LN("Press hoverboard button during a time given in hoverboard manual - press enter when done");
  waitKey('\r');
  
  LOG_INFO_LN("Calibrating...");
   
  /** 5s calibration */
  uint32_t loc_u32_time = millis();
  while(millis() - loc_u32_time < 7000)
  {
    _p_hoverboard->idleControl();
  }
  LOG_INFO_LN("Calibration done - press hoverboard button during a time given in hoverboard manual to exit calibration - press enter when done");
  waitKey('\r');
}

void HoverboardListenerExample::pollCmd(void)
{
  uint16_t loc_u16_cmdLength = 0;
  while(LOG_SERIAL.available())
  {
    if(loc_u16_cmdLength > MAX_CMD_LENGTH)
    {
       LOG_ERROR("Invalid command length");
       return;
    }
    _au8_rcvCmd[loc_u16_cmdLength++] = LOG_SERIAL.read();
  }

  if(loc_u16_cmdLength > 0)
  {
    handleCommand(_au8_rcvCmd, loc_u16_cmdLength);
  }
}

void HoverboardListenerExample::sendCmd(EFeedbackCmds arg_e_cmdType, uint8_t arg_au8_cmd[], uint8_t arg_u8_cmdLength)
{
  //TODO
}

void HoverboardListenerExample::handleCommand(uint8_t arg_u8_cmd[], uint16_t arg_u16_cmdLength)
{
  if(arg_u8_cmd[0] == POWER_ON && arg_u16_cmdLength == POWER_ON_CMD_LENGTH)
  {
    LOG_INFO_LN("POWER_ON");
    _p_hoverboard->powerOn();
  }
  else if(arg_u8_cmd[0] == POWER_OFF && arg_u16_cmdLength == POWER_OFF_CMD_LENGTH)
  {
    LOG_INFO_LN("POWER_OFF");
    _p_hoverboard->powerOff();
  }
  else if(arg_u8_cmd[0] == SET_SPEED && arg_u16_cmdLength == SET_SPEED_CMD_LENGTH)
  {
    /** Data in little endian */
    float _speed1, _speed2;
    uint32_t _rampUpDur;
    uint32_t temp_val = arg_u8_cmd[1] | 
      arg_u8_cmd[2] << 8  |
      arg_u8_cmd[3] << 16 |
      arg_u8_cmd[4] << 24;
      
    _speed1 = *(float*)(&temp_val);
    
    temp_val = arg_u8_cmd[5] | 
      arg_u8_cmd[6] << 8  |
      arg_u8_cmd[7] << 16 |
      arg_u8_cmd[8] << 24;

    _speed2 = *(float*)(&temp_val);

    _rampUpDur = arg_u8_cmd[9] | 
      arg_u8_cmd[10] << 8  |
      arg_u8_cmd[11] << 16 |
      arg_u8_cmd[12] << 24;

    if(_rampUpDur <= 0)
    {
      LOG_ERROR("Invalid ramp up duration given");
    }
    else
    {
      LOG_INFO_LN("SET_SPEED TO (%f, %f) in %d ms",
        _speed1,
        _speed2,
        _rampUpDur);
      _p_hoverboard->setSpeedAsync(_speed1, _speed2, _rampUpDur); 
    }
  }
  else if(arg_u8_cmd[0] == STOP && arg_u16_cmdLength == STOP_CMD_LENGTH)
  {
    LOG_INFO_LN("STOP TODO");
  }
  else
  {
    LOG_INFO_LN("Command %d of length %d not handled",
      arg_u8_cmd[0],
      arg_u16_cmdLength);
  }
}
