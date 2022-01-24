/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.settings;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.wpilibj.Filesystem;

public class Constants {
    public static final class Drivetrain {
        public static final double LEFT_FRONT_STEER_OFFSET = -Math.toRadians(239.4); // TODO: set these values
        public static final double RIGHT_FRONT_STEER_OFFSET = -Math.toRadians(278.1); // TODO: set these values
        public static final double LEFT_REAR_STEER_OFFSET = -Math.toRadians(119.0); // TODO: set these values
        public static final double RIGHT_REAR_STEER_OFFSET = -Math.toRadians(43); // TODO: set these values

        public static final double WHEELBASE_METERS = 0.7; // TODO: check this value
        public static final double TRACKWIDTH_METERS = 0.53; // TODO check this value

        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 3.0;
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk3 standard module using Falcon500s to
        // drive.
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight
         * line.
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
                * SdsModuleConfigurations.MK3_STANDARD.getDriveReduction()
                * SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;

        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
                / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_PER_SECOND = Math.PI;
    }

    public static final class Auto {
        public static double delayTime = 1;
        public static double driveTime = 1;
        // TODO: Tune these values
        public final static Gains kAutoPositionGains = new Gains(.69, 0, 0);
        public final static Gains kAutoThetaGains = new Gains(.69, 0, 0);
    }

    public static final class OI {
        public static final double kJoystickDeadband = 0.075;
    }

    public static final class Intake {
        public static final boolean INTAKE_MOTOR_INVERTED = false;
        public static double kForwardIntakeSpeed = 0.6;
        public static double kReverseIntakeSpeed = 0.3;
    }

    public static final class Shooter {
        public static final double kWaitPeriod = 0.25;

        public static final class Flywheel {
            public static final boolean LEFT_FLYWHEEL_MOTOR_INVERTED = false;
            public static final boolean FYLWHEEL_ENCODER_INVERTED = false;

            // TODO: Determine optimal speed
            public static final double targetSpeed = 4050;
            public static final double acceptableSpeed = 4000;

            public static final double RAMP_RATE = 0.3;

            // TODO: These came from Robot Characterization for Tucker (2020)
            // Voltage to overcome static friction
            public static final double kS = 0.146;
            // Voltage to hold at a given velocity while overcoming viscous drag
            public static final double kV = 0.132;
            // Voltage needed to induce a given acceleration
            public static final double kA = 0.0489;

            public static final double kMaxVoltage = 12.0;

            // TODO: Tune these values
            // Proportional gain
            public static final double kP = .001;
            // Integral gain
            public static final double kI = 0;
            // Derivative gain
            public static final double kD = .01;

            public final static Gains kShooterGains = new Gains(kP, kI, kD, 1023.0 / 20660.0, 300, 1.00);

            /**
             * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
             * or 3. Only the first two (0,1) are visible in web-based configuration.
             */
            public static final int kSlotIdx = 0;
            /**
             * Talon FX supports multiple (cascaded) PID loops. For now we just want the
             * primary one.
             */
            public static final int kPIDLoopIdx = 0;

            /**
             * Set to zero to skip waiting for confirmation, set to nonzero to wait and
             * report to DS if action fails.
             */
            public static final int kTimeoutMs = 30;

            /**
             * Motor neutral dead-band, set to the minimum 0.1%.
             */
            public final static double kNeutralDeadband = 0.001;

            public final static double kVoltageCompSaturation = 10;

            public final static int closedLoopTimeMs = 1;
        }

        public static final class Hood {
            public static final double kMaxHoodError = 1.0; // Degrees
            public static double hoodErrorAdjustment = 11.5;
        }

        public static final class Hopper {
            public static final boolean HOPPER_MOTOR_INVERTED = false;
            public static final double kHopperSpeed = 0.2;
            public static double rampTime = 2;
        }

        public static final class Indexer {
            public static final boolean LEFT_INDEXER_MOTOR_INVERTED = true;
            public static final double kIndexerSpeed = 0.25;
        }
    }

    public static final class Vision {
        public static final String NTtable = "limelight";
    }

    public static final class Sensors {
        public static final double ColorRange = 100;
    }

    public static final class Directories {
        public static final String deployDirectory = Filesystem.getDeployDirectory().getAbsolutePath();
        public static final String pathsDirectory = deployDirectory + "/paths/";
    }
}