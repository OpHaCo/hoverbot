#!/usr/bin/env python
# encoding: utf-8

'''
@file    motor_loop_monitor.py 
@author  Rémi Pincent - INRIA
@date    28/03/2017

 @brief Monitor motor loop with some motor graphs
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
  
import matplotlib.pyplot as plt
import time
import datetime
from collections import deque
import numpy as np
import random
import sys
import matplotlib.animation as animation
import logging

class MotorLoopMonitor():
    
    def __init__(self):
        global _instance 
        self._graphs={} 
        self._fig = plt.figure()
        self._fig.canvas.mpl_connect('key_press_event', self._keypress)
        self._start_display_time = None 
        self._pause_display = False
        # interactive graphs
        plt.ion()


    def clear_graphs(self) :
        for graph_name in self._graphs :
            for line_name in self._graphs[graph_name][1] :
                # clear time and y for each graph 
                self._graphs[graph_name][1][line_name] = (self._graphs[graph_name][1][line_name][0], [], [])
                
                # clear line data
                self._graphs[graph_name][1][line_name][0].set_xdata([])
                self._graphs[graph_name][1][line_name][0].set_ydata([])
            
        self.update_display() 
        

    def add_value(self, graph_name, y, line, tc=None):
        assert graph_name in self._graphs, 'Graph {} has not beed added - add_graph must be called'.format(graph_name)
        if tc is None : 
            tc = (datetime.datetime.now() - self._start_display_time).total_seconds()
        assert line in self._graphs[graph_name][1], 'line {} not in {} lines'.format(graph_name, line)
        
        curr_plot_line = self._graphs[graph_name][1][line][0]
        
        # do a copy of time and value for possible future use 
        time_values = self._graphs[graph_name][1][line][1]
        y_values = self._graphs[graph_name][1][line][2]
        time_values.append(tc)
        y_values.append(y)
        
        if not self._pause_display : 
            # update line, graph will be updated on update_display call
            curr_plot_line.set_xdata(np.append(curr_plot_line.get_xdata(), tc)) 
            curr_plot_line.set_ydata(np.append(curr_plot_line.get_ydata(), y)) 
        
            # update x and ylim to show all points:
            xmin = float("inf") 
            xmax = -float("inf") 
            ymin = float("inf") 
            ymax = -float("inf") 
            for curr_line_name in self._graphs[graph_name][1] : 
                curr_line = self._graphs[graph_name][1][curr_line_name][0] 
                if len(curr_line.get_xdata()) == 0 :
                    continue
                
                temp_val = curr_line.get_xdata().min() 
                if temp_val < xmin :
                    xmin = temp_val 
                    
                temp_val = curr_line.get_xdata().max() 
                if temp_val > xmax :
                    xmax =  temp_val 
                    
                temp_val = curr_line.get_ydata().min() 
                if curr_line.get_ydata().min() < ymin :
                    ymin = temp_val 
                    
                temp_val = curr_line.get_ydata().max() 
                if temp_val > ymax :
                    ymax =  temp_val 
                    
            self._graphs[graph_name][0].set_xlim(xmin - 0.5, xmax + 0.5)
            self._graphs[graph_name][0].set_ylim(ymin - 0.5, ymax + 0.5)  
        

    ''' 
    Display all plots. update_display must be called
    to update plot on data change
    '''
    def start_display(self):
        # improve spacing between graphs 
        self._fig.tight_layout()
        #  Do not forget this call, it displays graphs, plt.draw(),  
        #  self._fig.canvas.draw(), cannot replace it 
        plt.pause(0.001)   
        self.update_display()  
    
    
    def stop_display(self):
        plt.close()


    def pause_display(self):
        self._pause_display = True
    
    
    def resume_display(self):
        for graph_name in self._graphs :
            for line_name in self._graphs[graph_name][1] :
                # Update graphs with values added during pause 
                self._graphs[graph_name][1][line_name][0].set_xdata(np.asarray(self._graphs[graph_name][1][line_name][1]))
                self._graphs[graph_name][1][line_name][0].set_ydata(np.asarray(self._graphs[graph_name][1][line_name][2]))
            
        self.update_display() 
        self._pause_display = False
    
    '''
    Add line to given graph
    '''
    def add_line(self, graph_name, line_name, color):
        assert graph_name in self._graphs, 'Graph {} has not beed added - add_graph must be called'.format(graph_name)
        
        # Draw an empty plot with legend
        # shortcut : "--bo" = '--' linestyle, 'b' for blue, 'o' marker
        new_line, = self._graphs[graph_name][0].plot([], [], label=line_name, linestyle='--', marker='o', ms=0.8, color=color, linewidth=0.5)		
        
        #(line, x, y, color) 
        self._graphs[graph_name][1][line_name] = (new_line, [], [], color)
       
        # add legend for lines
        self._graphs[graph_name][0].legend(loc="upper right")  
        
        
    def add_graph(self, graph_name, ylabel, line_color, ylim=None):
        assert graph_name not in self._graphs, 'Graph {} has already been added - add_graph must not  be called twice'.format(graph_name)
        if self._start_display_time == None :
             self._start_display_time = datetime.datetime.now()

        # https://gist.github.com/LeoHuckvale/89683dc242f871c8e69b
        # now later you get a new subplot; change the geometry of the existing
        n = len(self._fig.axes)
        for i in range(n):
           self._fig.axes[i].change_geometry(n+1, 1, i+1)
        
        pl = self._fig.add_subplot(len(self._graphs) + 1, 1,len(self._graphs) + 1) 
        if ylim is not None : 
            pl.set_ylim(ylim)
        pl.set_xlabel('time(s)')
        pl.set_title(graph_name)
        
        # plot, lines 
        self._graphs[graph_name] = (pl, {}) 
        
        #add default graph
        self.add_line(graph_name, ylabel, line_color) 


    def update_display(self):
        # plt.pause(0.001) update graph but is very slow, 
        # plt.draw() do not update graphs 
        # a fast solution is to call these methods 
        self._fig.canvas.update()
        self._fig.canvas.flush_events()
        
    
    def _keypress(self, event):
        if event.key == 'c' :
            logging.debug('reset graphs') 
            self.clear_graphs() 
        elif event.key == 'p' :
            logging.debug('pause graphs') 
            self.pause_display() 
        elif event.key == 'r' :
            logging.debug('resume graphs') 
            self.resume_display() 
        

def motor_loop_test() :
    motor_loop = MotorLoopMonitor() 
    motor_loop.add_graph("graph1", 'y1')
    motor_loop.add_graph("graph2", 'y2')
    motor_loop.start_display() 
    time.sleep(10) 
    for i in range(40):
        motor_loop.add_value("graph1", random.randint(0,20) )
        motor_loop.add_value("graph2", random.randint(0,80) )
        time.sleep(0.1) 
    time.sleep(10) 


if __name__ == "__main__":
    sys.exit(motor_loop_test())

