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
import serial
import sys
import os
import select
from optparse import OptionParser
from commander import *
import struct
from enum import IntEnum

''' Hoverboard commands '''
class HoverboardCmd(Command):
    class CmdId(IntEnum):
        SET_SPEED = 0
        POWER_ON  = 1
        POWER_OFF = 2 
        STOP      = 3
    
    def __init__(self,quit_commands=['q','quit','exit'], help_commands=['help','?', 'h'], serial = None):
        Command.__init__(self, quit_commands, help_commands)
        self.serial = serial
        
    def do_set_speed(self, *args):
        usage = 'USAGE : set_speed speed1(float) speed2(float))' 
        ''' parse left and right motor speed'''
        if len(args) != 2 :
            raise Exception('2 arguments must be given\n{}'.format(usage))
        try :
            speed1 = float(args[0])
            speed2 = float(args[1])
        except Exception as ValueError :
           raise Exception('speed  must be a floating value\n{}'.format(usage))
        ''' send command to hoverboard in LE'''
        
        try :
            if self.serial and self.serial.is_open :
                data_to_send = struct.pack('<Bff', HoverboardCmd.CmdId.SET_SPEED, speed1, speed2)
                self.serial.write(data_to_send)
            else :
                raise Exception('cannot send command - uart not connected')
        except Exception as e:
            raise Exception('cannot write command to uart - {}'.format(e))
        return '> SET SPEED TO (speed1={}, speed2={})'.format(speed1, speed2) 
    
    def do_power_on(self, *args):
        usage = 'USAGE poweron'
        try :
            if self.serial and self.serial.is_open :
                data_to_send = struct.pack('B', HoverboardCmd.CmdId.POWER_ON)
                self.serial.write(data_to_send)
            else :
                raise Exception('cannot send command - uart not connected')
        except Exception as e:
            raise Exception('cannot write command to uart - {}'.format(e))
        return '> POWER_ON'

    def do_power_off(self, *args):
        usage = 'USAGE poweroff'
        try :
            if self.serial and self.serial.is_open :
                data_to_send = struct.pack('B', HoverboardCmd.CmdId.POWER_OFF)
                self.serial.write(data_to_send)
            else :
                raise Exception('cannot send command - uart not connected')
        except Exception as e:
            raise Exception('cannot write command to uart - {}'.format(e))
        return '> POWER_OFF'
    
    def do_stop(self, *args):
        usage = 'USAGE stop'
        try :
            if self.serial and self.serial.is_open :
                data_to_send = struct.pack('B', HoverboardCmd.CmdId.STOP)
                self.serial.write(data_to_send)
            else :
                raise Exception('cannot send command - uart not connected')
        except Exception as e:
            raise Exception('cannot write command to uart - {}'.format(e))
        return '> STOP'
    
''' Hoverboard interfacing threw UART '''
class HoverboardUART :

    def __init__(self, port):
        self.port = port
        ''' uart will be connected after a connectUART() call  - pass a None port to 
        not connect UART now '''
        self.serial = serial.Serial(
            port=None,
            baudrate=115200,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.SEVENBITS,
            timeout=1
        )
        self.serial.port = self.port 
        self.uart_term = Commander("HOVERBOT term", cmd_cb=HoverboardCmd(serial = self.serial))
        self.is_last_newline = True 
        
    def connectUART(self, nbTries) :
        self.uart_term.output_line("try to open port {}...".format(self.port), 'normal')

        while(1) :
            try :
                self.serial.open() 
                break

            except Exception as e :
                nbTries = nbTries - 1
                time.sleep(0.01)
                if nbTries == 0 :
                    raise 

        if self.serial.isOpen() :
            self.uart_term.output_line("Port {} opened".format(self.port), 'normal')
        else : 
            self.uart_term.output_line("Port {} not opened - exit".format(self.port), 'error')
            return 

        while(1) :
            self.loop()

    def loop(self) :
        self.readUART()
        self.captureUserCmds()
    
    def updateTerm(self):
        self.uart_term.loop() 
        

    def close(self) :
        if self.serial and self.serial.isOpen() :
            self.serial.close()
            self.serial = None

    def readUART(self) :
        try :
            # read all that is there or wait for one byte
            data = self.serial.read(self.serial.in_waiting or 1)
            if data:
                text = data.decode("utf-8")
                # Append char indicating some data have been received from hoverbot
                if self.is_last_newline :
                    text = '< ' + text 
                if text[-1:] == '\n':
                    self.is_last_newline = True
                    # Insert char indicating some data have been received from hoverbot
                    text = text[:-1].replace('\n', '\n< ') 
                    text = text + '\n'
                else :
                    text = text.replace('\n', '\n< ') 
                    self.is_last_newline = False
                self.uart_term.output_text(text, 'green') 
                 
        except OSError as e :
            self.uart_term.output_line("Error in stream... try to reconnect", 'error')
            if self.serial.isOpen :
                self.serial.close()
                self.connectUART(1000)

    def captureUserCmds(self) :
        return
        #print("todo")


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
