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
from motor_loop_monitor import MotorLoopMonitor 
import traceback
import warnings
import logging

    
''' Hoverboard interfacing threw UART '''
class HoverboardUART :

    def __init__(self, port):
        ''' uart will be connected after a connect_uart() call  - pass a None port to 
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
        self._motor_loop_monitor = MotorLoopMonitor() 
        self._motor_loop_monitor.add_graph('motor_1 PID debug', 'setpoint', 'm') 
        self._motor_loop_monitor.add_line('motor_1 PID debug', 'filtered_input', 'y') 
        self._motor_loop_monitor.add_line('motor_1 PID debug', 'input', 'c') 
        self._motor_loop_monitor.add_line('motor_1 PID debug', 'output', 'k') 
        self._motor_loop_monitor.add_line('motor_1 PID debug', 'error', 'r') 
        self._motor_loop_monitor.add_line('motor_1 PID debug', 'gyr', 'b') 
        self._motor_loop_monitor.add_graph('motor_2 PID debug', 'setpoint', 'm') 
        self._motor_loop_monitor.add_line('motor_2 PID debug', 'filtered_input', 'y') 
        self._motor_loop_monitor.add_line('motor_2 PID debug', 'input', 'c') 
        self._motor_loop_monitor.add_line('motor_2 PID debug', 'output', 'k') 
        self._motor_loop_monitor.add_line('motor_2 PID debug', 'error', 'r') 
        self._motor_loop_monitor.add_line('motor_2 PID debug', 'gyr', 'b') 
        # timecode set on hoverbot, on remote, use a relative timestamp 
        self._pid_debug_rel_tc = None
        self._uart_th = None 
        self._stop_uart_th = True 

        self._pid_data_lock = threading.Lock()
        self._new_pid_data = [] 
        self._display_pid = False
        
        
    def display_pid_plots(self):
        # start here capture of pid logs  
        self._motor_loop_monitor.start_display() 
        self._display_pid = True 
        
        
    def close_pid_plots(self):
        self._motor_loop_monitor.stop_display() 
        self._display_pid = False 
        self._pid_debug_rel_tc = None 


    def loop(self) :
        graph_updated = False 
        with self._pid_data_lock :
            if len(self._new_pid_data) > 0 :
                for new_data in self._new_pid_data : 
                    # Get a relative timestamp from now 
                    if self._pid_debug_rel_tc is None :
                        self._pid_debug_rel_tc = new_data[0]
                    # Update of plot data must be done in main thread. If not it doesn't work!
                    self._motor_loop_monitor.add_value('motor_{} PID debug'.format(new_data[1]), new_data[2], tc=(new_data[0] - self._pid_debug_rel_tc)/1000, line='filtered_input')
                    self._motor_loop_monitor.add_value('motor_{} PID debug'.format(new_data[1]), new_data[5], tc=(new_data[0] - self._pid_debug_rel_tc)/1000, line='input')
                    self._motor_loop_monitor.add_value('motor_{} PID debug'.format(new_data[1]), new_data[3], tc=(new_data[0] - self._pid_debug_rel_tc)/1000, line='setpoint')
                    self._motor_loop_monitor.add_value('motor_{} PID debug'.format(new_data[1]), new_data[4], tc=(new_data[0] - self._pid_debug_rel_tc)/1000, line='output')
                    self._motor_loop_monitor.add_value('motor_{} PID debug'.format(new_data[1]), new_data[3]-new_data[2], tc=(new_data[0] - self._pid_debug_rel_tc)/1000, line='error')
                    self._motor_loop_monitor.add_value('motor_{} PID debug'.format(new_data[1]), new_data[6], tc=(new_data[0] - self._pid_debug_rel_tc)/1000, line='gyr')
                graph_updated = True 
                self._new_pid_data.clear()
                
        # be sure UI updated out of any protected zone
        if self._display_pid : 
            # Updating display can take some 100ms 
            self._motor_loop_monitor.update_display() 
        self._motor_loop_monitor.update_display() 
        
        self._uart_term.update()
        
        
    def open(self):
        # run UART in a separate thread 
        self._uart_th = Thread(target=self._connect_uart, args=(1,))
        self._stop_uart_th = False 
        self._uart_th.daemon=True
        self._uart_th.start()
        self._start_term()


    def close(self) :
        if self._serial and self._serial.isOpen() :
            self._serial.close()
            self._serial = None
        if self._display_pid :
            self.close_pid_plots()
        self._stop_uart_th = True
        while  self._uart_th and self._uart_th.is_alive() : 
           time.sleep(0.1) 
        self._stop_term()


    def readUART(self) :
        try :
            # read all that is there or wait for one byte
            data = self._serial.read(self._serial.in_waiting or 1)
            if data:
                data = self.handle_hoverbot_data(data) 
                text = data.decode("utf-8")
                # Add timestamp in ms
                time_str = HoverboardUART.getTime() 
                
                # last char received was a new line, add it to beginning of new chars received 
                if self._is_last_newline :
                    text = '\n' + text 
                    self._is_last_newline = False
                if text[-1:] == '\n':
                    # remove last char => timestamp will be added to next log
                    text = text[:-1]
                    self._is_last_newline = True
                    
                # split lines to add timestamp to each line 
                if '\n' in text :
                    lines = text.split('\n')
                    first_index=0 
                    # First non empty string must not be added as a new line
                    if len(lines[0]) > 0 :	
                        self._uart_term.output_text(lines[0], 'green') 
                        first_index = 1 
                    for line in lines[first_index:] :
                        if len(line) > 0 : 
                            self._uart_term.output_line(time_str + ' < ' + line, 'green') 
                else:
                    self._uart_term.output_text(text, 'green') 
        except UnicodeDecodeError :
            logging.error("bad byte received")
            pass 
            
        except OSError as e :
            self._uart_term.output_line("Error in stream... try to reconnect", 'error')
            if self._serial.isOpen :
                self._serial.close()
                self._connect_uart(1000)
                
                
    '''
    Data received from UART are logged.
    Some specific data received from UART needs an additional handling
    ''' 
    def handle_hoverbot_data(self, data) :
        # Handle here all hoverboard specific commands 
        logging.debug('handle data') 
        cmd_index, data = self._detect_preamble(data) 
        
        if cmd_index != -1 : 
            logging.debug('preamble detected') 
            cmd_detected, data = self._detect_pid_log_cmd(data)  
            if cmd_detected :
                return data
            cmd_detected, data = self._detect_encoder_log_cmd(data)
            if cmd_detected :
                return data
        return data 
                
                
    def setSpeed(self, speed1, speed2, rampUpDur):
        try :
            if self._serial and self._serial.is_open :
                data_to_send = struct.pack('<BffI', HoverboardUART.HoverboardCmd.MasterCmdId.SET_SPEED, speed1, speed2, rampUpDur)
                self._serial.write(data_to_send)
                self._speed=(speed1, speed2) 
            else :
                raise Exception('cannot send command - uart not connected')
        except Exception as e:
            raise Exception('cannot write command to uart - {}'.format(e))
    
    
    def stop(self):
        try :
            if self._serial and self._serial.is_open :
                data_to_send = struct.pack('<B', HoverboardUART.HoverboardCmd.MasterCmdId.STOP)
                self._serial.write(data_to_send)
                self._speed = (0, 0) 
            else :
                raise Exception('cannot send command - uart not connected')
        except Exception as e:
            raise Exception('cannot write command to uart - {}'.format(e))
        
        
    def getTime() :
       return datetime.now().strftime("%H:%M:%S.%f")[:-3]  


    '''
    Return first preamble found
    '''
    def _detect_preamble(self, data):
        preamble_len = len(HoverboardUART.HoverboardCmd.PREAMBLE)
        # try to find a cmd preamble
        preamble_index = -1 
        
        # TODO : check incomplete preamble or multiple preamble   
        for preamble_id in range(len(data) - preamble_len + 1) :
            if HoverboardUART.HoverboardCmd.PREAMBLE == data[preamble_id:preamble_id + preamble_len] :
                preamble_index = preamble_id 
                break 
        if preamble_index != -1 : 
            # remove preamble from data
            data = data[:preamble_index] + data[preamble_index + preamble_len:] 
        else :
            # check for incomplete preamble
            # a frame will always have an incomplete preamble in its last bytes,
            # so iterate over incomplete preamble (from biggest incomplete to preamble of length 1)
            # and try to find it in last  data bytes
            logging.debug("check incomplete preamble") 
            for incomp_preamble_index in range(preamble_len - 1) :
                if len(data) - preamble_len + 1 + incomp_preamble_index < 0 :
                    continue
                if HoverboardUART.HoverboardCmd.PREAMBLE[:preamble_len-1 - incomp_preamble_index] == data[len(data) - preamble_len + 1 + incomp_preamble_index:] :
                    new_data = []
                    # +2 for command id
                    new_data = self._serial.read(incomp_preamble_index + 2)
                    if new_data :
                        data = data + new_data
                        if data[len(data) - preamble_len - 1:len(data) - 1 ] == HoverboardUART.HoverboardCmd.PREAMBLE :
                            # remove preamble from data
                            data = data[len(data) - 1:]
                            preamble_index = 0 
                    else :
                        raise Exception('unable to get missing preamble data') 
                    
        return preamble_index, data 


    def _detect_pid_log_cmd(self, data) :
        cmd_detected = False 
        while HoverboardUART.HoverboardCmd.SlaveCmdId.PID_LOG in data :
            cmd_detected = True 
            try:  
                cmd_index = data.index(HoverboardUART.HoverboardCmd.SlaveCmdId.PID_LOG)
                if cmd_index + HoverboardUART.HoverboardCmd.SlaveCmdLength.PID_LOG_LENGTH <= len(data) :
                    cmd_id,tc, motor_id, pid_input, pid_setpoint, pid_output, raw_input, gyr  = (struct.unpack('<BIBfffih', data[cmd_index:cmd_index + HoverboardUART.HoverboardCmd.SlaveCmdLength.PID_LOG_LENGTH]))
                    logging.info('debug pid frame : (tc={}, motor_id={}, pid_setpoint={}, pid_filtered_input={}, input={}, pid_output={}, gyr={})'.format(tc, motor_id, pid_setpoint, pid_input, raw_input, pid_output, gyr)) 
                    
                    if self._display_pid : 
                        with self._pid_data_lock :
                            self._new_pid_data.append((tc, motor_id+1, pid_input, pid_setpoint, pid_output, raw_input, gyr)) 
                    # remove handled command from data
                    data = data[:cmd_index] + data[cmd_index + HoverboardUART.HoverboardCmd.SlaveCmdLength.PID_LOG_LENGTH:] 
                else :
                    new_data = []
                    new_data = self._serial.read(cmd_index + HoverboardUART.HoverboardCmd.SlaveCmdLength.PID_LOG_LENGTH - len(data))
                    if new_data :
                        data = data + new_data
                    else :
                        time.sleep(0.01)
            except ValueError :
                # no more command to handle 
                pass

        return cmd_detected, data 
    
    
    def _detect_encoder_log_cmd(self, data) :
        cmd_detected = False 
        while HoverboardUART.HoverboardCmd.SlaveCmdId.ENCODER_LOG in data :
            cmd_detected = True 
            try:  
                cmd_index = data.index(HoverboardUART.HoverboardCmd.SlaveCmdId.ENCODER_LOG)
                if cmd_index + HoverboardUART.HoverboardCmd.SlaveCmdLength.ENCODER_LOG_LENGTH <= len(data) :
                    cmd_id, tc, motor_1_ticks, motor_2_ticks = (struct.unpack('<BIii', data[cmd_index:cmd_index + HoverboardUART.HoverboardCmd.SlaveCmdLength.ENCODER_LOG_LENGTH]))
                    logging.info('debug encoder frame : (tc={}, motor_1_ticks={}, motor_2_ticks={})'.format(tc, motor_1_ticks, motor_2_ticks)) 
                    
                    self._uart_term.output_line('{} < debug encoder frame : (tc={}, motor_1_ticks={}, motor_2_ticks={})'.format(HoverboardUART.getTime(), tc, motor_1_ticks, motor_2_ticks), 'green') 
                    # remove handled command from data
                    data = data[:cmd_index] + data[cmd_index + HoverboardUART.HoverboardCmd.SlaveCmdLength.ENCODER_LOG_LENGTH:] 
                else :
                    new_data = []
                    new_data = self._serial.read(cmd_index + HoverboardUART.HoverboardCmd.SlaveCmdLength.ENCODER_LOG_LENGTH - len(data))
                    if new_data :
                        data = data + new_data
                    else :
                        time.sleep(0.01)
            except ValueError :
                # no more command to handle 
                pass

        return cmd_detected, data 

    def _start_term(self):
        #Update terminal 
        self._uart_term.start_loop() 
        
        
    def _stop_term(self):
        #Update terminal 
        self._uart_term.stop_loop() 


    def _connect_uart(self, nbTries) :
        self._uart_term.output_line("try to open port {}...".format(self._serial._port), 'normal')

        while True and not self._stop_uart_th :
            try :
                self._serial.open() 
                break

            except Exception as e :
                nbTries = nbTries - 1
                time.sleep(0.01)
                if nbTries == 0 :
                    raise Exception("cannot open port {}".format(self._serial._port)) 

        if self._serial.isOpen() :
            self._uart_term.output_line("Port {} opened".format(self._serial._port), 'normal')
        else : 
            self._uart_term.output_line("Port {} not opened - exit".format(self._serial._port), 'error')
            return 

        self._serial.reset_input_buffer()
        while True and not self._stop_uart_th :
            self.readUART()


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
        PREAMBLE=b'\xAB\xCD\xEF\x00'
        class MasterCmdId(IntEnum):
            SET_SPEED = 0
            POWER_ON  = 1
            POWER_OFF = 2 
            STOP      = 3
            GET_TICKS = 4 
            
            
        class SlaveCmdId(IntEnum):
            PID_LOG       = 1 
            ENCODER_LOG   = 2 
            
            
        class SlaveCmdLength(IntEnum):
            PID_LOG_LENGTH   = 24
            ENCODER_LOG_LENGTH   = 13
        
        
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
            except ValueError as e:
               raise Exception('Invalid argument given\n{}'.format(usage))
            self._outer._uart_term.output_line(HoverboardUART.getTime() + ' > SET SPEED TO (speed1={}, speed2={} in {} ms)'.format(speed1, speed2, rampUpDur))
            ''' send command to hoverboard in LE'''
            self._outer.setSpeed(speed1, speed2, rampUpDur); 
        
        
        def do_power_on(self, *args):
            usage = 'USAGE poweron'
            try :
                if self._outer._serial and self._outer._serial.is_open :
                    self._outer._uart_term.output_line(HoverboardUART.getTime() + ' > POWER_ON')
                    data_to_send = struct.pack('B', HoverboardUART.HoverboardCmd.MasterCmdId.POWER_ON)
                    self._outer._serial.write(data_to_send)
                    #TODO : get speed from hoverbot sensors
                    self._outer._speed = (0, 0) 
                else :
                    raise Exception('cannot send command - uart not connected')
            except Exception as e:
                raise Exception('cannot write command to uart - {}'.format(e))


        def do_power_off(self, *args):
            usage = 'USAGE poweroff'
            try :
                if self._outer._serial and self._outer._serial.is_open :
                    self._outer._uart_term.output_line(HoverboardUART.getTime() + ' > POWER_OFF')
                    data_to_send = struct.pack('B', HoverboardUART.HoverboardCmd.MasterCmdId.POWER_OFF)
                    self._outer._serial.write(data_to_send)
                    #TODO : get speed from hoverbot sensors
                    self._outer._speed = (0, 0) 
                else :
                    raise Exception('cannot send command - uart not connected')
            except Exception as e:
                raise Exception('cannot write command to uart - {}'.format(e))
        
        
        def do_stop(self, *args):
            usage = 'USAGE stop'
            try :
                if self._outer._serial and self._outer._serial.is_open :
                    self._outer._uart_term.output_line(HoverboardUART.getTime() + ' > STOP')
                    data_to_send = struct.pack('B', HoverboardUART.HoverboardCmd.MasterCmdId.STOP)
                    self._outer._serial.write(data_to_send)
                else :
                    raise Exception('cannot send command - uart not connected')
            except Exception as e:
                raise Exception('cannot write command to uart - {}'.format(e))
        
        
        def do_start_keypad_control(self, *args):
            usage = 'USAGE : start keypad control using numeric keypad'
            self._outer._uart_term.output_line(HoverboardUART.getTime() + ' NOW in manual keypad control')
            self._is_keypad_control = True
        
        
        def do_stop_keypad_control(self, *args):
            usage = 'USAGE : stop keypad control using numeric keypad'
            self._outer._uart_term.output_line(HoverboardUART.getTime() + ' EXITING manual keypad control')
            self._is_keypad_control = False
        
        
        def do_get_ticks(self, *args):
            usage = 'USAGE get_ticks'
            try :
                if self._outer._serial and self._outer._serial.is_open :
                    self._outer._uart_term.output_line(HoverboardUART.getTime() + ' > GET_TICKS')
                    data_to_send = struct.pack('B', HoverboardUART.HoverboardCmd.MasterCmdId.GET_TICKS)
                    self._outer._serial.write(data_to_send)
                else :
                    raise Exception('cannot send command - uart not connected')
            except Exception as e:
                raise Exception('cannot write command to uart - {}'.format(e))


# ALL warnings must be removed as it will be output to stdout
# => bad 
def fxn():
    warnings.warn("deprecated", DeprecationWarning)

def main(argv=None):

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        fxn()
       
        # Use root logger 
        hdlr = logging.FileHandler('/var/tmp/hoverbot.log')
        formatter = logging.Formatter('%(asctime)s %(filename)s %(message)s')
        hdlr.setFormatter(formatter)
        logging.getLogger().addHandler(hdlr) 
        logging.getLogger().setLevel(logging.DEBUG) 
        
        logging.info('Hoverbot uart control')
        
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
            def handle_uart():
                hoverboardUART._connect_uart(1)
            t=Thread(target=handle_uart)
            t.daemon=True
            t.start()
            
            hoverboardUART.open()
            hoverboardUART.display_pid_plots()
            while True :
                hoverboardUART.loop()
                #time.sleep(0.1) 
                
            if hoverboardUART :
                hoverboardUART.close()
                hoverboardUART = None 

        except KeyboardInterrupt as k:
            if hoverboardUART :
                hoverboardUART.close()
                hoverboardUART = None 
            sys.stderr.write("program will exit\nBye!\n")
            return 0

        except Exception as e:
            if hoverboardUART :
                hoverboardUART.close()
                hoverboardUART = None 
            sys.stderr.write("Exception in main thread : {} {}\n".format(e, e.__class__.__name__))
            traceback.print_exception(*sys.exc_info()) 
            return 2

        finally :
            # do not close hoverboarUART here, in order to display some data to stdout (and not in hoverbot terminal, object must be closed) 
            pass 

if __name__ == "__main__":
    sys.exit(main())
