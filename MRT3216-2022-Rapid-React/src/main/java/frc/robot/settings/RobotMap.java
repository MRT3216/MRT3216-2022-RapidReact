/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.settings;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

/**
 * Add your docs here.
 */
public class RobotMap {
    public final static class ROBOT {
        public static class DRIVETRAIN {
            public static final int LEFT_FRONT_DRIVE =  1;
            public static final int LEFT_FRONT_ANGLE = 2;
            public static final int RIGHT_FRONT_DRIVE =  3;
            public static final int RIGHT_FRONT_ANGLE = 4;
            public static final int LEFT_REAR_DRIVE =  5;
            public static final int LEFT_REAR_ANGLE = 6;
            public static final int RIGHT_REAR_DRIVE =  7;
            public static final int RIGHT_REAR_ANGLE = 8;
            public static final int LEFT_FRONT_CANCODER =  15;
            public static final int RIGHT_FRONT_CANCODER =  16;
            public static final int LEFT_REAR_CANCODER =  17;
            public static final int RIGHT_REAR_CANCODER =  18;
        }

        public static class SHOOTER {
            public static final int LEFT_FLYWHEEL = 9;
            public static final int RIGHT_FLYWHEEL = 10;
            public static final int LEFT_INDEXER = 11;
            public static final int RIGHT_INDEXER = 12;
            public static final int HOPPER = 13;
            public static final int INTAKE = 14;
            public static final int HOOD_PWM_PORT = 1;
        }

        public static class INTAKE {
            public static final int INTAKE = 14;
        }

        public static class PNEUMATICS {
            public static final int INTAKE_FORWARD = 0;
            public static final int INTAKE_REVERSE = 1;
        }

        public static class SENSORS {
            public static final I2C.Port colorSensor = I2C.Port.kOnboard;
            public static final I2C.Port navx = I2C.Port.kOnboard;
        }
    }

    public final static class DRIVE_STATION {
        public static final int USB_XBOX_CONTROLLER = 0;
        public static final int USB_JOYSTICK = 1;
    }
}
