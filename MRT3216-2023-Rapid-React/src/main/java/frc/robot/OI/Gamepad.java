package frc.robot.OI;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Gamepad extends XboxController {
    /** Gamepad Button and Axis Mapping ***************************************/
    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int BUTTON_LB = 5;
    public static final int BUTTON_RB = 6;
    public static final int BUTTON_BACK = 7;
    public static final int BUTTON_START = 8;
    public static final int BUTTON_LEFT_JOY = 9;
    public static final int BUTTON_RIGHT_JOY = 10;

    /** Axis Mapping for a single joystick ************************************/
    public static final int RIGHT_JOY_X_AXIS = 4;
    public static final int RIGHT_JOY_Y_AXIS = 5;
    public static final int RIGHT_TRIGGER = 3;
    public static final int LEFT_TRIGGER = 2;
    public static final int LEFT_JOY_X_AXIS = 0;
    public static final int LEFT_JOY_Y_AXIS = 1;

    /** Button Declarations ***************************************************/
    public final JoystickButton A = new JoystickButton(this, BUTTON_A);
    public final JoystickButton B = new JoystickButton(this, BUTTON_B);
    public final JoystickButton X = new JoystickButton(this, BUTTON_X);
    public final JoystickButton Y = new JoystickButton(this, BUTTON_Y);
    public final JoystickButton LB = new JoystickButton(this, BUTTON_LB);
    public final JoystickButton RB = new JoystickButton(this, BUTTON_RB);
    public final JoystickButton Back = new JoystickButton(this, BUTTON_BACK);
    public final JoystickButton Start = new JoystickButton(this, BUTTON_START);
    public final JoystickButton LeftJoy = new JoystickButton(this, BUTTON_LEFT_JOY);
    public final JoystickButton RightJoy = new JoystickButton(this, BUTTON_RIGHT_JOY);

    public Gamepad(final int port) {
        super(port);
    }
}