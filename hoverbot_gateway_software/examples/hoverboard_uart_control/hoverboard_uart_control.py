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
from python_terminal import *

''' Hoverboard commands '''
class HoverboardCmd(Command):
    def do_echo(self, *args):
        '''echo - Just echos all arguments'''
        return ' '.join(args)
    def do_raise(self, *args):
        raise Exception('Some Error')

''' Hoverboard interfacing threw UART '''
class HoverboardUART :

    def __init__(self, port):
        self.port = port
        self.serial = None 
        self.uartTerm = Commander("HOVERBOT term", cmd_cb=HoverboardCmd())
        
    def connectUART(self, nbTries) :
        print("try to open port {}...".format(self.port))

        while(1) :
            try :
                # configure the serial connections (the parameters differs on the device you are connecting to)
                self.serial = serial.Serial(
                    port=self.port,
                    baudrate=115200,
                    parity=serial.PARITY_ODD,
                    stopbits=serial.STOPBITS_TWO,
                    bytesize=serial.SEVENBITS,
                    timeout=1
                )
                break

            except Exception as e :
                nbTries = nbTries - 1
                time.sleep(0.01)
                if nbTries == 0 :
                    raise 

        if self.serial.isOpen() :
            pass
            #print("Port {} opened".format(self.port))
        else : 
            sys.stderr.write("Port {} not opened - exit".format(self.port))
            return 

        while(1) :
            self.loop()

    def loop(self) :
        self.readUART()
        self.captureUserCmds()
    
    def updateTerm(self):
        self.uartTerm.loop() 
        

    def close(self) :
        if self.serial and self.serial.isOpen() :
            self.serial.close()
            self.serial = None

    def readUART(self) :
        try :
            # read all that is there or wait for one byte
            data = self.serial.read(self.serial.in_waiting or 1)
            if data:
                #sys.stdout.write(data.decode('ascii')) 
                self.uartTerm.output(data.decode('ascii'), 'green') 

        except OSError as e :
            sys.stderr.write("Error in stream... try to reconnect\n")
            if self.serial.isOpen :
                self.serial.close()
                self.connectUART(self.port, 1000)

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
