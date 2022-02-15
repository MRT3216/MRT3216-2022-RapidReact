/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.settings;

import edu.wpi.first.wpilibj.I2C;

/**
 * Add your docs here.
 */
//todo fix all can values in this class
public class RobotMap {
    public final static class ROBOT {
        public static class DRIVETRAIN {
            public static final int LEFT_FRONT_DRIVE = 1;
            public static final int LEFT_FRONT_ANGLE = 2;
            public static final int RIGHT_FRONT_DRIVE = 3;
            public static final int RIGHT_FRONT_ANGLE = 4;
            public static final int LEFT_REAR_DRIVE = 5;
            public static final int LEFT_REAR_ANGLE = 6;
            public static final int RIGHT_REAR_DRIVE = 7;
            public static final int RIGHT_REAR_ANGLE = 8;
            public static final int LEFT_FRONT_CANCODER = 15;
            public static final int RIGHT_FRONT_CANCODER = 16;
            public static final int LEFT_REAR_CANCODER = 17;
            public static final int RIGHT_REAR_CANCODER = 18;
        }

        public static class SHOOTER {
            public static final int LEFT_FLYWHEEL = 9;
            public static final int RIGHT_FLYWHEEL = 10;
            public static final int LEFT_INDEXER = 11;
            public static final int RIGHT_INDEXER = 12;
            public static final int HOOD_PWM_PORT = 1;
            //todo change this to the correct CAN value
            public static final int HOPPER_MOTOR = -1;
        }

        public static class INTAKE {
            public static final int INTAKE_MOTOR = 14;
            public static final int HOPPER = 13;
        }

        public static class PNEUMATICS {
            public static final int INTAKE_FORWARD = 0;
            public static final int INTAKE_REVERSE = 1;
        }

        public static class SENSORS {
            public static final I2C.Port MUX_PORT = I2C.Port.kMXP;
            public static final int TRAY_COLOR = 0;
            public static final int SHOOTER_COLOR = 1;
            public static final int NAVX = 2;
            public static final int BREAK_BEAM = 0;

        }
    }

    public final static class DRIVE_STATION {
        public static final int USB_XBOX_CONTROLLER = 0;
        public static final int USB_JOYSTICK = 1;
    }
}
