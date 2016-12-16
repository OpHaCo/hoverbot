/******************************************************************************
 * 
 * @file    hoverboard_listener.h
 * @author  Rémi Pincent - INRIA
 * @date    01/12/2016
 *
 * @brief Hoverboard listener class. Get listener events.
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
#ifndef HOVERBOARD_LISTENER_H
#define HOVERBOARD_LISTENER_H

/**************************************************************************
 * Include Files
 **************************************************************************/

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
class HoverboardListener
{
  public : 
    /**
     * Speed changed - units rad/s
     **/
    virtual void onSpeedChanged(float arg_f_motor1Speed, float arg_f_motor2Speed) = 0;
    
    /***
     * setSpeed async callback 
     **/
    virtual void onSpeedApplied(void) = 0;
    
    /***
     * poweron async callback 
     **/
    virtual void onPowerOn(void) = 0;
    
    /***
     * poweroff async callback 
     **/
    virtual void onPowerOff(void) = 0;
};

#endif /** HOVERBOARD_LISTENER_H **/
