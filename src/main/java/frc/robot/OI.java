/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

    /**
     * This class is the glue that binds the controls on the physical operator
     * interface to the commands and command groups that allow control of the robot.
     */
    public class OI { 
    Joystick drivePad;
    Joystick toolPad;

    JoystickButton bButton;
    /**
     * Initializes the joystick objects 
     */
    public OI()
    {
        drivePad = new Joystick (GamepadConstants.DRIVE_USB_PORT);
        toolPad = new Joystick (GamepadConstants.TOOL_USB_PORT);
    }

    /**
     * Used to return the drivePad's right joystick y-axis value 
     * 
     * @return Returns y-value from right joystick on the drivePad
     */
    public double getDriveRightY ()
    {
        double joy = drivePad.getRawAxis(GamepadConstants.RIGHT_ANALOG_Y);
        if(Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }
    /**
     * Used to return the toolPad's right joystick y-axis value 
     * 
     * @return Returns y-value from right joystick on the toolPad
     */
    public double getToolRightY ()
    {
        double joy = toolPad.getRawAxis(GamepadConstants.RIGHT_ANALOG_Y);
        if(Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }
    /**
     * Used to return the drivePad's left joystick y-axis value 
     * 
     * @return Returns y-value from left joystick on the drivePad
     */
    public double getDriveLeftY ()
    {
        double joy = drivePad.getRawAxis(GamepadConstants.LEFT_ANALOG_Y);
        if(Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }
    /**
     * Used to return the toolPad's left joystick y-axis value 
     * 
     * @return Returns y-value from left joystick on the toolPad
     */
    public double getToolLeftY ()
    {
        double joy = toolPad.getRawAxis(GamepadConstants.LEFT_ANALOG_Y);
        if(Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }
    /**
     * Used to return the drivePad's right joystick x-axis value 
     * 
     * @return Returns x-value from right joystick on the drivePad
     */
    public double getDriveRightX()
    {
        double joy = drivePad.getRawAxis(GamepadConstants.RIGHT_ANALOG_X);
        if(Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }
    /**
     * Used to return the toolPad's right joystick x-axis value 
     * 
     * @return Returns x-value from right joystick on the toolPad
     */
    public double getToolRightX()
    {
        double joy = toolPad.getRawAxis(GamepadConstants.RIGHT_ANALOG_X);
        if(Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }
    /**
     * Used to return the drivePad's left joystick x-axis value 
     * 
     * @return Returns x-value from left joystick on the drivePad
     */
    public double getDriveLeftX()
    {
        double joy = drivePad.getRawAxis(GamepadConstants.LEFT_ANALOG_X);
        if(Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }
    /**
     * Used to return the toolPad's left joystick x-axis value 
     * 
     * @return Returns x-value from left joystick on the toolPad
     */
    public double getToolLeftX()
    {
        double joy = toolPad.getRawAxis(GamepadConstants.LEFT_ANALOG_X);
        if(Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }

    //Drive DPAD up
    public boolean getDriveDPadUp() {
        if (drivePad.getPOV(0) == GamepadConstants.DPAD_UP) {
            return true;
        } else {
            return false;
        }
    }

    //Drive DPAD right
    public boolean getDriveDPadRight() {
        if (drivePad.getPOV(0) == GamepadConstants.DPAD_RIGHT) {
            return true;
        } else {
            return false;
        }
    }

    //Drive DPAD down
    public boolean getDriveDPadDown() {
        if (drivePad.getPOV(0) == GamepadConstants.DPAD_DOWN) {
            return true;
        } else {
            return false;
        }
    }

    //Drive DPAD left
    public boolean getDriveDPadLeft() {
        if (drivePad.getPOV(0) == GamepadConstants.DPAD_LEFT) {
            return true;
        } else {
            return false;
        }
    }

    //Tool DPAD up
    public boolean getToolDPadUp() {
        if (toolPad.getPOV(0) == GamepadConstants.DPAD_UP) {
            return true;
        } else {
            return false;
        }
    }

    //Tool DPAD right
    public boolean getToolDPadRight() {
        if (toolPad.getPOV(0) == GamepadConstants.DPAD_RIGHT) {
            return true;
        } else {
            return false;
        }
    }

    //Tool DPAD down
    public boolean getToolDPadDown() {
        if (toolPad.getPOV(0) == GamepadConstants.DPAD_DOWN) {
            return true;
        } else {
            return false;
        }
    }

    //Tool DPAD left
    public boolean getToolDPadLeft() {
        if (toolPad.getPOV(0) == GamepadConstants.DPAD_LEFT) {
            return true;
        } else {
            return false;
        }
    }

    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveRightTrigger()
    {
        return drivePad.getRawButton(GamepadConstants.RIGHT_TRIGGER);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolRightTrigger()
    {
        return toolPad.getRawButton(GamepadConstants.RIGHT_TRIGGER);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveLeftTrigger()
    {
        return drivePad.getRawButton(GamepadConstants.LEFT_TRIGGER);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolLeftTrigger()
    {
        return toolPad.getRawButton(GamepadConstants.LEFT_TRIGGER);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveLeftBumper()
    {
        return drivePad.getRawButton(GamepadConstants.LEFT_BUMPER);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolLeftBumper()
    {
        return toolPad.getRawButton(GamepadConstants.LEFT_BUMPER);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveRightBumper()
    {
        return drivePad.getRawButton(GamepadConstants.RIGHT_BUMPER);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolRightBumper()
    {
        return toolPad.getRawButton(GamepadConstants.RIGHT_BUMPER);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolXButton()
    {
        return toolPad.getRawButton(GamepadConstants.X_BUTTON);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolAButton()
    {
        return toolPad.getRawButton(GamepadConstants.A_BUTTON);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolBButton()
    {
        return toolPad.getRawButton(GamepadConstants.B_BUTTON);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolYButton()
    {
        return toolPad.getRawButton(GamepadConstants.Y_BUTTON);
    }

    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveXButton()
    {
        return drivePad.getRawButton(GamepadConstants.X_BUTTON);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveAButton()
    {
        return drivePad.getRawButton(GamepadConstants.A_BUTTON);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveBButton()
    {
        return drivePad.getRawButton(GamepadConstants.B_BUTTON);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveYButton()
    {
        return drivePad.getRawButton(GamepadConstants.Y_BUTTON);
    }

    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveBackButton()
    {
        return toolPad.getRawButton(GamepadConstants.BACK_BUTTON);
    }


    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolStartButton()
    {
        return toolPad.getRawButton(GamepadConstants.START_BUTTON);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolBackButton()
    {
        return toolPad.getRawButton(GamepadConstants.BACK_BUTTON);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolLeftButton()
    {
        return toolPad.getRawButton(GamepadConstants.LEFT_ANALOG_BUTTON);
    }
    /** 
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolRightButton()
    {
        return toolPad.getRawButton(GamepadConstants.RIGHT_ANALOG_BUTTON);
    }

    public void driveDistance () {
        //bButton.whenPressed(new DriveDistance(50,5,0.5,0));
    }

    public boolean getDriveStartButton() {
    // TODO Auto-generated method stub
    return drivePad.getRawButton(GamepadConstants.START_BUTTON);
    }
}