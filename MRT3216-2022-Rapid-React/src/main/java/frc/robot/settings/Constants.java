/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.settings;

import com.revrobotics.CANSparkMaxLowLevel;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public final class Constants {
    public static CANSparkMaxLowLevel.MotorType kBrushed = CANSparkMaxLowLevel.MotorType.kBrushed;
    public static CANSparkMaxLowLevel.MotorType kBrusheless = CANSparkMaxLowLevel.MotorType.kBrushless;

    public enum Ball {
        ALLIANCE, OPPONENT, NONE
    }

    public static final class Drivetrain {
        // TODO: set these values
        public static final double LEFT_FRONT_STEER_OFFSET = -Math.toRadians(19.33);
        public static final double RIGHT_FRONT_STEER_OFFSET = -Math.toRadians(276.86);
        public static final double LEFT_REAR_STEER_OFFSET = -Math.toRadians(113.02);
        public static final double RIGHT_REAR_STEER_OFFSET = -Math.toRadians(47.62);

        public static final double WHEELBASE_METERS = 0.5461;
        public static final double TRACKWIDTH_METERS = 0.5588;

        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 12.0;
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
        // Proportional gain
        public static final double kPositionP = 0.001;
        // Integral gain
        public static final double kPositionI = 0;
        // Derivative gain
        public static final double kPositionD = 0;
        // Proportional gain
        public static final double kThetaP = 20;
        // Integral gain
        public static final double kThetaI = 0;
        // Derivative gain
        public static final double kThetaD = 0.6;

        public static final Gains kAutoPositionGains = new Gains(kPositionP, kPositionI, kThetaD);
        public static final Gains kAutoThetaGains = new Gains(kThetaP, kThetaI, kThetaD);

        public static final double kMaxTurnError = .1; // degrees
        public static final double kMaxTurnRateError = 1; // Degrees per second

        public static final double kMaxTurnErrorAuto = 5;
        public static final double kMaxTurnRateErrorAuto = 5;

        public static final int kMaxFetchVelocity = 8;
        public static final int kMaxFetchAcc = kMaxFetchVelocity / 2;

        public static final double kStartDelayTime = 0;
        public static final double kDriveToShootDelay = 0; // seconds
        public static final double kMaxShootTime = 2; // seconds
    }

    public static final class OI {
        public static final double kJoystickDeadband = 0.1;
        public static final double kTranslationExpo = 75;
        public static final double kRotationnExpo = 75;
    }

    public static final class Intake {
        public static final boolean INTAKE_MOTOR_INVERTED = false;
        public final static double kVoltageCompSaturation = 10;
        public static final double kForwardIntakeSpeed = 0.65;
        public static final double kReverseIntakeSpeed = 0.3;
    }

    public static final class Shooter {
        public static final double kWaitPeriod = 0;
        public static final double kDistanceAdjustmentInMeters = 0;

        public static final class Flywheel {
            public static final boolean FLYWHEEL_MOTOR_INVERTED = true;
            public static final boolean FYLWHEEL_ENCODER_INVERTED = false;
            public static final double ballShotfilterThreshold = -250;
            public static final double ballShotDebounceTime = 0.5;

            public static final double targetShootingRPM = 2345;
            public static final double acceptableShootingRPM = 2300;
            public static final double targetEjectRPM = 1500;
            public static final double acceptableEjectRPM = 1450;

            public static final double RAMP_RATE = 0.3;

            // Proportional gain
            public static final double kP = .0085355;
            // Integral gainc
            public static final double kI = 0;
            // Derivative gain
            public static final double kD = 0;
            public static final double kF = 0.0575;
            public static final int kIzone = 300;
            public static final double kPeakOutput = 1.0;
            public static final int kSensorUnitsPerRotation = 2048;
            /*
             * These came from characterization but aren't currently used
             * public static final double kS = 0.67572;
             * public static final double kV = 0.14379;
             * public static final double kA = 0.014485;
             */

            public static final Gains kShooterGains = new Gains(kP, kI, kD, kF, kIzone, kPeakOutput);

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
            public static final boolean HOOD_MOTOR_INVERTED = true;
            public static final double kMaxHoodError = 1.0; // Degrees

            // Proportional gain
            public static final double kP = 20;
            // Integral gainc
            public static final double kI = 0;
            // Derivative gain
            public static final double kD = 0.1;
            public static final double kS = 0.12344;
            public static final double kG = -0.10972;
            public static final double kV = 1.4653;
            public static final double kA = 2.2813;
            public static final double kMaxVelocity = 3;
            public static final double kMaxAcceleration = 5;
            public static final double kEncoderDistancePerPulse = 8192;
            public static final double kArmOffsetRads = -3.41;
            public final static double kVoltageCompSaturation = 10;

            public static final double hoodForwardLimit = -.9;
            public static final double hoodReverseLimit = 0;

            public static final double hoodStowedAngle = Units.degreesToRadians(5);
        }

        public static final class Hopper {
            public static final boolean HOPPER_MOTOR_INVERTED = true;
            public static final double kHopperSpeed = 0.7;
            public final static double kVoltageCompSaturation = 10;
            public static double rampTime = 2;
        }

        public static final class Indexer {
            public static final boolean INDEXER_MOTOR_INVERTED = false;
            public final static double kVoltageCompSaturation = 10;
            public static final double shootingRPM = 1500;
            public static final double indexingRPM = 1000;
            public static final double RAMP_RATE = 0.3;

            /**
             * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
             * or 3. Only the first two (0,1) are visible in web-based configuration.
             */
            public static final int kSlotIdx = 0;

            // Proportional gain
            public static final double kP = 0.00000038864;
            // Integral gainc
            public static final double kI = 0;
            // Derivative gain
            public static final double kD = 0;
            public static final double kF = 0.0572;
            public static final int kIzone = 0;
            public static final double kPeakOutput = 1.0;
            public static final int kSensorUnitsPerRotation = 2048;
            /*
             * These came from characterization but aren't currently used
             * public static final double kS = 0.66782;
             * public static final double kV = 0.017064;
             * public static final double kA = 0.00056598;
             */

            /**
             * Motor neutral dead-band, set to the minimum 0.1%.
             */
            public final static double kNeutralDeadband = 0.001;

            public final static int closedLoopTimeMs = 1;

            public static final Gains kIndexerGains = new Gains(kP, kI, kD, kF, kIzone, kPeakOutput);
        }
    }

    public static final class LimeLight {
        public static final String NTtable = "limelight";

        // modes:
        // 0 = use the LED Mode set in the current pipeline
        // 1 = force off
        // 2 = force blink
        // 3 = force on
        public enum LEDMode {
            PIPELINE, OFF, BLINK, ON
        }

        // modes:
        // 0 = vision processor
        // 1 = driver camera
        public enum CameraMode {
            VISION, DRIVER
        }

        // Set stream:
        // 0 = Standard - Side-by-side streams if a webcam is attached to Limelight
        // 1 = PiP Main - The secondary camera stream is placed in the lower-right
        // corner of the primary camera stream
        // 2 = PiP Secondary - The primary camera stream is placed in the lower-right
        // corner of the secondary camera stream
        public enum CameraStream {
            Standard, PiPMain, PiPSecondary
        }
    }

    public static final class Sensors {
        public static final double ColorRange = 150;

    }

    public static final class Directories {
        public static final String deployDirectory = Filesystem.getDeployDirectory().getAbsolutePath();
        public static final String pathsDirectory = deployDirectory + "/paths/";
    }

    public static final class Projectile {
        // Camera Constants
        public final static double kCameraHeight = 0.721; // The vertical distance from the center of the camera to the
                                                          // ground.
        public final static double kCameraViewAngle = Units.degreesToRadians(40); // Angle (rads) of the center of POV
                                                                                  // of the camera from horizontal.

        public final static double kCameraOffsetFromFrame = 0.04; // The horizontal distance from the center of the
        // camera to the front of the robot.

        // Shooter Constants
        public final static double kShooterOffsetFromFrame = 0.2667; // The horizontal distance from the front of the
                                                                     // robot to the shooter.
        public final static double kShooterHeight = 0.688; // The vertical distance from the ground to the shooter.

        // Goal Constants
        public final static double kTargetHeight = 2.642; // The vertical distance from the vision tape to the ground.
        public final static double kTargetGoalHorizontalOffest = 0.61; // The horiztonal distance from the vison tape
                                                                       // to the center of the goal;

        // Projectile Constant
        public final static double kMaxProjectileHeight = 3.25; // The maximum height from the ground of the
                                                                // projectile.
        public final static double kMinPorjectileHoriztonalVelocity = 0.75; // The minimum horizontal velocity of the
                                                                            // projectile. (m/s)
        public final static double kAccelDueToGravity = 9.8; // The acceleration due to gravity (m/s^s)
        public final static double kTargetHeightFromCamera = kTargetHeight - kCameraHeight; // The vertical distance
        // from the camera to the shooter.

        public final static double kInitVerticalVelocity = Math
                .sqrt(2 * kAccelDueToGravity * (kMaxProjectileHeight - kShooterHeight)); // The initial vertical
                                                                                         // velocity of the projectile.
    }

    public static final class Climber {
        public final static double kVoltageCompSaturation = 10;
    }
}