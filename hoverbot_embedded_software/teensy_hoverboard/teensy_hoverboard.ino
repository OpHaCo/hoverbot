static int16_t currPos = 0;

#define LOG_SERIAL                        Serial
#define DAUGHTER_BOARD_SERIAL             Serial2
#define DAUGHTER_BOARD_SERIAL_SIM_L       Serial1
#define DAUGHTER_BOARD_SERIAL_SIM_R       Serial3

void setup() {
   LOG_SERIAL.begin(115200);
   
   DAUGHTER_BOARD_SERIAL.begin (26300, SERIAL_9N1);        // 9 bits mode
   DAUGHTER_BOARD_SERIAL_SIM_L.begin (26300, SERIAL_9N1);  // 9 bits mode
   DAUGHTER_BOARD_SERIAL_SIM_R.begin (26300, SERIAL_9N1);  // 9 bits mode
   
   LOG_SERIAL.println("start");
}

void loop() {
  echoGoForward(0);
  //daughterEcho();
  //printCurrentPos();
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
        uint16_t rFrame[6] = {256, (-readPos) & 0xff, (-readPos) >> 8 & 0xff, (-readPos) & 0xff, (-readPos) >> 8 & 0xff, 85};

        for(uint8_t index = 0; index < 6; index++)
        {
          DAUGHTER_BOARD_SERIAL_SIM_R.write9bit(lFrame[index]);
          DAUGHTER_BOARD_SERIAL_SIM_L.write9bit(rFrame[index]); 
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
