/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.ballardRobotics.frc;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 *
 * @author Lemonzap
 */
public class ToggleButton{
    
    JoystickButton button;
    boolean isPressed = false;
    private boolean isToggled = false;
    
    public ToggleButton(JoystickButton receivedButton){
        this.button = receivedButton;
    }
    
    public boolean isToggled(){
        if(this.button.get() && !isPressed){
            isPressed = true;
            isToggled = !isToggled;
        }
        if(!this.button.get()){
            isPressed = false;
        }
            return isToggled;
    }
    
    
        
}
