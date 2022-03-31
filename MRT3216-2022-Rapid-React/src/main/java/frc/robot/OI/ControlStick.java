
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.OI;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Add your docs here.
 */
public class ControlStick extends Joystick {

    /** Gamepad Button and Axis Mapping ***************************************/
    public static final int BUTTON_TRIGGER = 1;
    public static final int BUTTON_2 = 2;
    public static final int BUTTON_3 = 3;
    public static final int BUTTON_4 = 4;
    public static final int BUTTON_5 = 5;
    public static final int BUTTON_6 = 6;
    public static final int BUTTON_7 = 7;
    public static final int BUTTON_8 = 8;
    public static final int BUTTON_9 = 9;
    public static final int BUTTON_10 = 10;
    public static final int BUTTON_11 = 11;
    public static final int BUTTON_12 = 12;

    /**
     * Axis Mapping for a single joystick
     ************************************/
    public static final int JOYSTICK_Y_AXIS = 1;
    public static final int JOYSTICK_THROTTLE_AXIS = 3;

    /**
     * Button Declarations
     ***************************************************/
    public JoystickButton Trigger = new JoystickButton(this, BUTTON_TRIGGER);
    public JoystickButton button2 = new JoystickButton(this, BUTTON_2);
    public JoystickButton button3 = new JoystickButton(this, BUTTON_3);
    public JoystickButton button4 = new JoystickButton(this, BUTTON_4);
    public JoystickButton button5 = new JoystickButton(this, BUTTON_5);
    public JoystickButton button6 = new JoystickButton(this, BUTTON_6);
    public JoystickButton button7 = new JoystickButton(this, BUTTON_7);
    public JoystickButton button8 = new JoystickButton(this, BUTTON_8);
    public JoystickButton button9 = new JoystickButton(this, BUTTON_9);
    public JoystickButton button10 = new JoystickButton(this, BUTTON_10);
    public JoystickButton button11 = new JoystickButton(this, BUTTON_11);
    public JoystickButton button12 = new JoystickButton(this, BUTTON_12);

    public ControlStick(int port) {
        super(port);
    }
}
