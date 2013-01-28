/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ballardRobotics.frc;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotMain extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    SpeedController motor;
    Encoder motorEnc;
    PIDController pid;
    double p;
    double i;
    double d;
    double f;
    double setPoint;
    Joystick joy1;
    Joystick joy2;
    ToggleButton PIDToggle;
    JoystickButton lTrigger;
    ToggleButton motorToggle;
    JoystickButton rTrigger;
    Encoder magEnc;
    
    public void robotInit() {
        joy2 = new Joystick(2);
        joy1 = new Joystick(1);
        motorToggle = new ToggleButton(rTrigger = new JoystickButton(joy1, 1));
        PIDToggle = new ToggleButton(lTrigger = new JoystickButton(joy2, 1));
        //setPoint = 5;
        motorEnc = new Encoder(3, 4);
        magEnc = new Encoder(5, 6);
        motorEnc.setDistancePerPulse(.0027777777);
        magEnc.setDistancePerPulse(.0027777777);
        motorEnc.start();
        magEnc.start();
        motorEnc.reset();
        magEnc.reset();
        motorEnc.setPIDSourceParameter(Encoder.PIDSourceParameter.kRate);
        magEnc.setPIDSourceParameter(Encoder.PIDSourceParameter.kRate);
        
        //magEnc = new AnalogChannel(1);
        
        motor = new Talon(5);
        
        pid = new PIDController(p, i, d, f, motorEnc, motor);
        pid.setContinuous();
        pid.setInputRange(0, 10000);
        
        
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        
        SmartDashboard.putNumber("EncSpeed", motorEnc.getRate());
        SmartDashboard.putNumber("EncValue", motorEnc.getDistance());
        SmartDashboard.putNumber("magEncSpeed", magEnc.getRate());
        SmartDashboard.putNumber("magEncDistance", magEnc.getDistance());
        SmartDashboard.putNumber("EncRate", motorEnc.getRate());
        
       // SmartDashboard.putNumber("MagEncVal", magEnc.getValue());
        
        pid.setSetpoint(setPoint);
        SmartDashboard.putNumber("motorValue", motor.get());
        
        p = (joy2.getThrottle() + 1)/2;
        i = (joy1.getZ() + 1)/2;
        d = 0;
        f = 0;
         
        
        SmartDashboard.putNumber("p", p);
        SmartDashboard.putNumber("i", i);
        
       if(PIDToggle.isToggled() && !motorToggle.isToggled()){
            if(!pid.isEnable()){
                pid.enable();
                SmartDashboard.putString("PID State", "enabled");
            }     
        }else{
            if(pid.isEnable()){
                pid.disable();
                SmartDashboard.putString("PID State", "disabled");
            }
        }
        if(motorToggle.isToggled() && !PIDToggle.isToggled()){
            motor.set(joy1.getY());
            SmartDashboard.putString("Motor Control State","enabled");
        }else{
            SmartDashboard.putString("Motor Control State","disabled");
        }
        setPoint = (joy2.getY() * 5) + 5;
        SmartDashboard.putNumber("setPoint", setPoint);
        pid.setPID(p, i, d);
        
        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
