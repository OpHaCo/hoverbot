static int16_t currPos = 0;

#define LOG_SERIAL                        Serial
#define DAUGHTER_BOARD_SERIAL             Serial2
#define DAUGHTER_BOARD_SERIAL_SIM_L       Serial1
#define DAUGHTER_BOARD_SERIAL_SIM_R       Serial3

static uint16_t _calValue = 0;
static float _integralVal = 0;

void setup() {
   LOG_SERIAL.begin(115200);
   
   DAUGHTER_BOARD_SERIAL.begin (26300, SERIAL_9N1);        // 9 bits mode
   DAUGHTER_BOARD_SERIAL_SIM_L.begin (26300, SERIAL_9N1);  // 9 bits mode
   DAUGHTER_BOARD_SERIAL_SIM_R.begin (26300, SERIAL_9N1);  // 9 bits mode
   
   LOG_SERIAL.println("start");
}

void loop() {
  uint16_t targetSpeed = 200;
  Serial.print("Applying speed of ");
  Serial.println(targetSpeed);
  setSpeedTarget(targetSpeed);

  uint32_t startTime = millis();

  Serial.println("Apply idle control");
  /** keep speed during 10s */
  while(millis() - startTime < 10000)
    idleControl();

  Serial.println("stop");
  setSpeedTarget(-targetSpeed);

  Serial.println("Apply idle control");
  startTime = millis();
  /** Stopped during 1s */
  while(millis() - startTime < 1000)
    idleControl();
}

/** 
 *  echo value read from a single daughterboard to 
 *  motherboard left and right UART
 */
void daughterEcho(void)
{
  if (DAUGHTER_BOARD_SERIAL.available())
  {
    uint16_t data = DAUGHTER_BOARD_SERIAL.read();
    DAUGHTER_BOARD_SERIAL_SIM_R.write9bit(data);
    DAUGHTER_BOARD_SERIAL_SIM_L.write9bit(data);
  }
}

/** 
 *  Read current position from a single daughterboard
 */
void printCurrentPos(void)
{
    if (DAUGHTER_BOARD_SERIAL.available()) {
      int16_t readPos = 0;

      /** begin of frame */
      if(DAUGHTER_BOARD_SERIAL.read() == 256)
      { 
        /** get angle LSB (8 bits) */
        while (!DAUGHTER_BOARD_SERIAL.available()){};
        /** ninth bits not used */
        readPos |= DAUGHTER_BOARD_SERIAL.read() & 0xFF;

        /** get angle MSB (8 bits) */
        while (!DAUGHTER_BOARD_SERIAL.available()){};
        /** ninth bits not used */
        readPos |= (DAUGHTER_BOARD_SERIAL.read() & 0xFF) << 8;
        
        if(readPos != currPos)
        {
          currPos = readPos;
          LOG_SERIAL.println(readPos);
        }
      }
      else
      {
      }
  }
  else
  {
    //LOG_SERIAL.println("nothing to read");
  }
}

void stop(int16_t tar)
{
  LOG_SERIAL.print(millis()); 
  LOG_SERIAL.print(": integral ");
  LOG_SERIAL.println(_integralVal);

  /** TODO */
}

/** Keep current speed */
void idleControl(void)
{
  uint16_t lFrame[6] = {256, _calValue & 0xff, _calValue >> 8 & 0xff, _calValue & 0xff, _calValue >> 8 & 0xff, 85};
  uint16_t rFrame[6] = {256, (-_calValue) & 0xff, (-_calValue) >> 8 & 0xff, (-_calValue) & 0xff, (-_calValue) >> 8 & 0xff, 85};
  
  for(uint8_t index = 0; index < 6; index++)
  {
    DAUGHTER_BOARD_SERIAL_SIM_R.write9bit(rFrame[index]);
    DAUGHTER_BOARD_SERIAL_SIM_L.write9bit(lFrame[index]); 
  }
}


/** 
 * Integral control of speed
 */
void setSpeedTarget(int16_t speed)
{  
  static const float RAMP_UP_FACTOR = 0.1;

  uint16_t  speedRampUp = (speed*RAMP_UP_FACTOR);      
  uint16_t lFrame[6] = {256, speedRampUp & 0xff, speedRampUp >> 8 & 0xff, speedRampUp & 0xff, speedRampUp >> 8 & 0xff, 85};
  uint16_t rFrame[6] = {256, (-speedRampUp) & 0xff, (-speedRampUp) >> 8 & 0xff, (-speedRampUp) & 0xff, (-speedRampUp) >> 8 & 0xff, 85};
  
  uint32_t timeStart = millis();
  while((millis() - timeStart) < (1/RAMP_UP_FACTOR)*1000)
  {
    for(uint8_t index = 0; index < 6; index++)
    {
      DAUGHTER_BOARD_SERIAL_SIM_R.write9bit(rFrame[index]);
      DAUGHTER_BOARD_SERIAL_SIM_L.write9bit(lFrame[index]); 
    }
  }
  _integralVal = (millis() - timeStart)*speedRampUp;
}

/** 
 *  Read current position from a single daughterboard and 
 *  control both uard to go forward
 */
void echoGoForward(uint16_t speed)
{
    if (DAUGHTER_BOARD_SERIAL.available()) {
      int16_t readPos = 0;

      /** begin of frame */
      if(DAUGHTER_BOARD_SERIAL.read() == 256)
      { 
        /** get angle LSB (8 bits) */
        while (!DAUGHTER_BOARD_SERIAL.available()){};
        /** ninth bits not used */
        readPos |= DAUGHTER_BOARD_SERIAL.read() & 0xFF;

        /** get angle MSB (8 bits) */
        while (!DAUGHTER_BOARD_SERIAL.available()){};
        /** ninth bits not used */
        readPos |= (DAUGHTER_BOARD_SERIAL.read() & 0xFF) << 8;

        
        
        /** echo current frame on both uart with forward direction */
        uint16_t lFrame[6] = {256, readPos & 0xff, readPos >> 8 & 0xff, readPos & 0xff, readPos >> 8 & 0xff, 85};
        //readPos = 10;
        uint16_t rFrame[6] = {256, (-readPos) & 0xff, (-readPos) >> 8 & 0xff, (-readPos) & 0xff, (-readPos) >> 8 & 0xff, 85};

        for(uint8_t index = 0; index < 6; index++)
        {
          DAUGHTER_BOARD_SERIAL_SIM_R.write9bit(rFrame[index]);
          DAUGHTER_BOARD_SERIAL_SIM_L.write9bit(lFrame[index]); 
        }
        
        if(readPos != currPos)
        {
          currPos = readPos;
          LOG_SERIAL.println(readPos);
        }
      }
      else
      {
      }
  }
  else
  {
    //LOG_SERIAL.println("nothing to read");
  }  
}
