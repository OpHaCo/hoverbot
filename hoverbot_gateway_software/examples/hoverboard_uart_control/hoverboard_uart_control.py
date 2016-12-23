#!/usr/bin/env python
# encoding: utf-8

'''
@file    hoverboard_uart_control.py 
@author  Rémi Pincent - INRIA
@date    30/11/2016

 @brief Control hoverboard from UART. It displays two split panes that :
     * get logs from hoverboard
     * give user means to send some commands to hoverboard

 Project : hoverbot
 Contact:  Rémi Pincent - remi.pincent@inria.fr

 Revision History:
     https://github.com/OpHaCo/hoverbot.git 

  LICENSE :
      hoverbot (c) by Rémi Pincent
      hoverbot is licensed under a
      Creative Commons Attribution-NonCommercial 3.0 Unported License.

  You should have received a copy of the license along with this
  work.  If not, see <http://creativecommons.org/licenses/by-nc/3.0/>.
  '''

import time
from datetime import datetime
import serial
import sys
import os
import select
from optparse import OptionParser
from commander import *
import struct
from enum import IntEnum

    
''' Hoverboard interfacing threw UART '''
class HoverboardUART :

    def __init__(self, port):
        ''' uart will be connected after a connectUART() call  - pass a None port to 
        not connect UART now '''
        self._serial = serial.Serial(
            port=None,
            baudrate=500000,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.SEVENBITS,
            timeout=1
        )
        self._serial.port = port 
        self._cmds = HoverboardUART.HoverboardCmd(self) 
        self._uart_term = Commander("HOVERBOT term", input_edit=HoverboardUART.InputHoverboard(self), cmd_cb=self._cmds)
        self._is_last_newline = True 
        self._speed = (0, 0) 
        
    def connectUART(self, nbTries) :
        self._uart_term.output_line("try to open port {}...".format(self._serial._port), 'normal')

        while(1) :
            try :
                self._serial.open() 
                break

            except Exception as e :
                nbTries = nbTries - 1
                time.sleep(0.01)
                if nbTries == 0 :
                    raise 

        if self._serial.isOpen() :
            self._uart_term.output_line("Port {} opened".format(self._serial._port), 'normal')
        else : 
            self._uart_term.output_line("Port {} not opened - exit".format(self._serial._port), 'error')
            return 

        while(1) :
            self.loop()

    def loop(self) :
        self.readUART()
    
    def updateTerm(self):
        self._uart_term.loop() 
        

    def close(self) :
        if self._serial and self._serial.isOpen() :
            self._serial.close()
            self._serial = None

    def readUART(self) :
        try :
            # read all that is there or wait for one byte
            data = self._serial.read(self._serial.in_waiting or 1)
            if data:
                text = data.decode("utf-8")
                # Add timestamp in ms
                time_str = HoverboardUART.getTime() 
                # Append char indicating some data have been received from hoverbot
                if self._is_last_newline :
                    text = time_str + ' < ' + text 
                if text[-1:] == '\n':
                    self._is_last_newline = True
                    # Insert char indicating some data have been received from hoverbot
                    text = text[:-1].replace('\n', '\n' + time_str + ' < ') 
                    text = text + '\n'
                else :
                    text = text.replace('\n', '\n' + time_str + ' < ') 
                    self._is_last_newline = False
                self._uart_term.output_text(text, 'green') 
                 
        except OSError as e :
            self._uart_term.output_line("Error in stream... try to reconnect", 'error')
            if self._serial.isOpen :
                self._serial.close()
                self.connectUART(1000)
                
    def setSpeed(self, speed1, speed2, rampUpDur):
        try :
            if self._serial and self._serial.is_open :
                data_to_send = struct.pack('<BffI', HoverboardUART.HoverboardCmd.CmdId.SET_SPEED, speed1, speed2, rampUpDur)
                self._serial.write(data_to_send)
                self._speed=(speed1, speed2) 
            else :
                raise Exception('cannot send command - uart not connected')
        except Exception as e:
            raise Exception('cannot write command to uart - {}'.format(e))
    
    def stop(self):
        try :
            if self._serial and self._serial.is_open :
                data_to_send = struct.pack('<B', HoverboardUART.HoverboardCmd.CmdId.STOP)
                self._serial.write(data_to_send)
                self._speed = (0, 0) 
            else :
                raise Exception('cannot send command - uart not connected')
        except Exception as e:
            raise Exception('cannot write command to uart - {}'.format(e))
        
    def getTime() :
       return datetime.now().strftime("%H:%M:%S.%f")[:-3]  


    ''' INNER CLASSES '''
    class InputHoverboard(Input) :
        
        def __init__(self, outer, got_focus=None):
            self._outer = outer 
            super().__init__(got_focus)
            
        def keypress(self, size, key):
            # Catch here command shortcuts 
            # Switch control mode : using numeric keypad or writing commands
            if key == 'x' :
                if self._outer._cmds._is_keypad_control : 
                    self._outer._uart_term.output_line(self._outer._cmds.do_stop_keypad_control())
                else : 
                    self._outer._uart_term.output_line(self._outer._cmds.do_start_keypad_control())
                return
            
            # Numeric keypad control
            elif key == '8' and self._outer._cmds._is_keypad_control :
                #Go in forward direction incresing speed
                return self._outer.setSpeed(self._outer._speed[0] - 50, self._outer._speed[1] - 50, 1000)
            elif key == '4' and self._outer._cmds._is_keypad_control :
                #Go left
                return self._outer.setSpeed(self._outer._speed[0] + 40, self._outer._speed[1] - 40, 1000)
            elif key == '6' and self._outer._cmds._is_keypad_control :
                #Go right
                return self._outer.setSpeed(self._outer._speed[0] - 40, self._outer._speed[1] + 40, 1000)
            elif key == '2' and self._outer._cmds._is_keypad_control :
                #Go in backward direction
                return self._outer.setSpeed(self._outer._speed[0] + 50, self._outer._speed[1] + 50, 1000)
            elif key == '5' and self._outer._cmds._is_keypad_control :
                #Stop
                return self._outer.stop();

            return super().keypress(size, key)
            
    ''' Hoverboard commands '''
    class HoverboardCmd(Command):
        class CmdId(IntEnum):
            SET_SPEED = 0
            POWER_ON  = 1
            POWER_OFF = 2 
            STOP      = 3
        
        def __init__(self, outer, quit_commands=['q','quit','exit'], help_commands=['help','?', 'h']):
            Command.__init__(self, quit_commands, help_commands)
            self._outer = outer
            self._is_keypad_control = False
            
        def do_set_speed(self, *args):
            usage = 'USAGE : set_speed speed1(float) speed2(float) ramp_up_duration(uint32)' 
            ''' parse left, right motor speed and ramp up duration'''
            if len(args) != 3 :
                raise Exception('3 arguments must be given\n{}'.format(usage))
            try :
                speed1 = float(args[0])
                speed2 = float(args[1])
                rampUpDur = int(args[2])
            except Exception as ValueError :
               raise Exception('Invalid argument given\n{}'.format(usage))
            ''' send command to hoverboard in LE'''
            self._outer.setSpeed(speed1, speed2, rampUpDur); 
            return HoverboardUART.getTime() + ' > SET SPEED TO (speed1={}, speed2={} in {} ms)'.format(speed1, speed2, rampUpDur) 
        
        def do_power_on(self, *args):
            usage = 'USAGE poweron'
            try :
                if self._outer._serial and self._outer._serial.is_open :
                    data_to_send = struct.pack('B', HoverboardUART.HoverboardCmd.CmdId.POWER_ON)
                    self._outer._serial.write(data_to_send)
                    #TODO : get speed from hoverbot sensors
                    self._outer._speed = (0, 0) 
                else :
                    raise Exception('cannot send command - uart not connected')
            except Exception as e:
                raise Exception('cannot write command to uart - {}'.format(e))
            return HoverboardUART.getTime() + ' > POWER_ON'

        def do_power_off(self, *args):
            usage = 'USAGE poweroff'
            try :
                if self._outer._serial and self._outer._serial.is_open :
                    data_to_send = struct.pack('B', HoverboardUART.HoverboardCmd.CmdId.POWER_OFF)
                    self._outer._serial.write(data_to_send)
                    #TODO : get speed from hoverbot sensors
                    self._outer._speed = (0, 0) 
                else :
                    raise Exception('cannot send command - uart not connected')
            except Exception as e:
                raise Exception('cannot write command to uart - {}'.format(e))
            return HoverboardUART.getTime() + ' POWER_OFF'
        
        def do_stop(self, *args):
            usage = 'USAGE stop'
            try :
                if self._outer._serial and self._outer._serial.is_open :
                    data_to_send = struct.pack('B', HoverboardUART.HoverboardCmd.CmdId.STOP)
                    self._outer._serial.write(data_to_send)
                else :
                    raise Exception('cannot send command - uart not connected')
            except Exception as e:
                raise Exception('cannot write command to uart - {}'.format(e))
            return  HoverboardUART.getTime() + ' STOP'
        
        def do_start_keypad_control(self, *args):
            usage = 'USAGE : start keypad control using numeric keypad'
            self._is_keypad_control = True
            return 'NOW in manual keypad control'
        
        def do_stop_keypad_control(self, *args):
            usage = 'USAGE : stop keypad control using numeric keypad'
            self._is_keypad_control = False
            return 'EXITING manual keypad control'


def main(argv=None):
    hoverboardUART = None
    if argv is None:
        argv = sys.argv[1:]
    try:
        # setup option parser
        parser = OptionParser()
        parser.add_option("-p", "--port", dest="port", default=None, help="Port of hoverboard Teensy UART (over FTDI)")
        (opts, args) = parser.parse_args(argv)

        if not opts.port :
            parser.error("A valid port must be given\n")
            sys.exit(2)
            
        hoverboardUART = HoverboardUART(opts.port)
        
        # run UART in a separate thread 
        def run():
            hoverboardUART.connectUART(1)
        t=Thread(target=run)
        t.daemon=True
        t.start()
        
        while(1) :
            #pass
            hoverboardUART.updateTerm()

    except KeyboardInterrupt as k:
        sys.stderr.write("program will exit\nBye!\n")
        return 0

    except Exception as e:
        sys.stderr.write("Exception in main thread : {} {}\n".format(e, e.__class__.__name__))
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(exc_type, fname, exc_tb.tb_lineno)
        return 2

    finally :
        if hoverboardUART :
            hoverboardUART.close()

if __name__ == "__main__":
    sys.exit(main())
