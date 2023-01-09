package frc.robot.settings;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.settings.Constants.Shooter.Flywheel;
import frc.robot.settings.Constants.Shooter.Hopper;
import frc.robot.settings.Constants.Shooter.Indexer;

public class Configurations {
    private TalonFXConfiguration hopperMotorConfiguration;
    private TalonFXConfiguration indexerMotorConfiguration;
    private TalonFXConfiguration flywheelMotorConfiguration;
    private TalonFXConfiguration swerveDriveMotorConfiguration;
    private TalonFXConfiguration swerveAngleMotorConfiguration;
    private static Configurations instance;

    private Configurations() {
    }

    public static Configurations getInstance() {
        if (instance == null) {
            instance = new Configurations();
        }

        return instance;
    }

    public TalonFXConfiguration getHopperMotorConfiguration() {
        if (hopperMotorConfiguration == null) {
            hopperMotorConfiguration = new TalonFXConfiguration();
            hopperMotorConfiguration.voltageCompSaturation = Hopper.kVoltageCompSaturation;
        }

        return hopperMotorConfiguration;
    }

    public TalonFXConfiguration getIndexerMotorConfiguration() {
        if (indexerMotorConfiguration == null) {
            indexerMotorConfiguration = new TalonFXConfiguration();
            indexerMotorConfiguration.voltageCompSaturation = Indexer.kVoltageCompSaturation;
            indexerMotorConfiguration.slot0.kF = Indexer.kIndexerGains.kF;
            indexerMotorConfiguration.slot0.kP = Indexer.kIndexerGains.kP;
            indexerMotorConfiguration.slot0.kI = Indexer.kIndexerGains.kI;
            indexerMotorConfiguration.slot0.kD = Indexer.kIndexerGains.kD;
            indexerMotorConfiguration.slot0.integralZone = Indexer.kIndexerGains.kIzone;
            indexerMotorConfiguration.nominalOutputForward = 0;
            indexerMotorConfiguration.nominalOutputReverse = 0;
            indexerMotorConfiguration.peakOutputForward = 1;
            indexerMotorConfiguration.peakOutputReverse = -1;

            indexerMotorConfiguration.slot0.closedLoopPeakOutput = Indexer.kIndexerGains.kPeakOutput;
            indexerMotorConfiguration.closedloopRamp = Indexer.RAMP_RATE;
            indexerMotorConfiguration.neutralDeadband = Indexer.kNeutralDeadband;
            indexerMotorConfiguration.slot0.closedLoopPeriod = Indexer.closedLoopTimeMs;
            indexerMotorConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor
                    .toFeedbackDevice();
        }

        return indexerMotorConfiguration;
    }

    public TalonFXConfiguration getFlywheelMotorConfiguration() {
        if (flywheelMotorConfiguration == null) {
            flywheelMotorConfiguration = new TalonFXConfiguration();
            flywheelMotorConfiguration.voltageCompSaturation = Flywheel.kVoltageCompSaturation;
            flywheelMotorConfiguration.slot0.kF = Flywheel.kShooterGains.kF;
            flywheelMotorConfiguration.slot0.kP = Flywheel.kShooterGains.kP;
            flywheelMotorConfiguration.slot0.kI = Flywheel.kShooterGains.kI;
            flywheelMotorConfiguration.slot0.kD = Flywheel.kShooterGains.kD;
            flywheelMotorConfiguration.slot0.integralZone = Flywheel.kShooterGains.kIzone;
            flywheelMotorConfiguration.nominalOutputForward = 0;
            flywheelMotorConfiguration.nominalOutputReverse = 0;
            flywheelMotorConfiguration.peakOutputForward = 1;
            flywheelMotorConfiguration.peakOutputReverse = -1;

            flywheelMotorConfiguration.slot0.closedLoopPeakOutput = Flywheel.kShooterGains.kPeakOutput;
            flywheelMotorConfiguration.closedloopRamp = Flywheel.RAMP_RATE;
            flywheelMotorConfiguration.neutralDeadband = Flywheel.kNeutralDeadband;
            flywheelMotorConfiguration.slot0.closedLoopPeriod = Flywheel.closedLoopTimeMs;
            flywheelMotorConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor
                    .toFeedbackDevice();
        }

        return flywheelMotorConfiguration;
    }

    public TalonFXConfiguration getSwerveAngleMotorConfiguration() {
        if (swerveAngleMotorConfiguration == null) {
            swerveAngleMotorConfiguration = new TalonFXConfiguration();
        }

        return swerveAngleMotorConfiguration;
    }

    public static class COTSFalconSwerveConstants {
        public final double wheelDiameter;
        public final double wheelCircumference;
        public final double angleGearRatio;
        public final double driveGearRatio;
        public final double angleKP;
        public final double angleKI;
        public final double angleKD;
        public final double angleKF;
        public final boolean driveMotorInvert;
        public final boolean angleMotorInvert;
        public final boolean canCoderInvert;

        public COTSFalconSwerveConstants(double wheelDiameter, double angleGearRatio, double driveGearRatio, double angleKP, double angleKI, double angleKD, double angleKF, boolean driveMotorInvert, boolean angleMotorInvert, boolean canCoderInvert){
            this.wheelDiameter = wheelDiameter;
            this.wheelCircumference = wheelDiameter * Math.PI;
            this.angleGearRatio = angleGearRatio;
            this.driveGearRatio = driveGearRatio;
            this.angleKP = angleKP;
            this.angleKI = angleKI;
            this.angleKD = angleKD;
            this.angleKF = angleKF;
            this.driveMotorInvert = driveMotorInvert;
            this.angleMotorInvert = angleMotorInvert;
            this.canCoderInvert = canCoderInvert;
        }

        /** Swerve Drive Specialties - MK3 Module*/
        public static COTSFalconSwerveConstants SDSMK3(double driveGearRatio){
            double wheelDiameter = Units.inchesToMeters(4.0);

            /** 12.8 : 1 */
            double angleGearRatio = (12.8 / 1.0);

            double angleKP = 0.2;
            double angleKI = 0.0;
            double angleKD = 0.0;
            double angleKF = 0.0;

            boolean driveMotorInvert = false;
            boolean angleMotorInvert = false;
            boolean canCoderInvert = false;
            return new COTSFalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, canCoderInvert);
        }

        /** Swerve Drive Specialties - MK4 Module*/
        public COTSFalconSwerveConstants SDSMK4(double driveGearRatio){
            double wheelDiameter = Units.inchesToMeters(4.0);

            /** 12.8 : 1 */
            double angleGearRatio = (12.8 / 1.0);

            double angleKP = 0.2;
            double angleKI = 0.0;
            double angleKD = 0.0;
            double angleKF = 0.0;

            boolean driveMotorInvert = false;
            boolean angleMotorInvert = false;
            boolean canCoderInvert = false;
            return new COTSFalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, canCoderInvert);
        }

        /** Swerve Drive Specialties - MK4i Module*/
        public COTSFalconSwerveConstants SDSMK4i(double driveGearRatio){
            double wheelDiameter = Units.inchesToMeters(4.0);

            /** (150 / 7) : 1 */
            double angleGearRatio = ((150.0 / 7.0) / 1.0);

            double angleKP = 0.3;
            double angleKI = 0.0;
            double angleKD = 0.0;
            double angleKF = 0.0;

            boolean driveMotorInvert = false;
            boolean angleMotorInvert = true;
            boolean canCoderInvert = false;
            return new COTSFalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, canCoderInvert);
        }

        /* Drive Gear Ratios for all supported modules */
        public class driveGearRatios{
            /* SDS MK3 */
            /** SDS MK3 - 8.16 : 1 */
            public static final double SDSMK3_Standard = (8.16 / 1.0);
            /** SDS MK3 - 6.86 : 1 */
            public static final double SDSMK3_Fast = (6.86 / 1.0);

            /* SDS MK4 */
            /** SDS MK4 - 8.14 : 1 */
            public static final double SDSMK4_L1 = (8.14 / 1.0);
            /** SDS MK4 - 6.75 : 1 */
            public static final double SDSMK4_L2 = (6.75 / 1.0);
            /** SDS MK4 - 6.12 : 1 */
            public static final double SDSMK4_L3 = (6.12 / 1.0);
            /** SDS MK4 - 5.14 : 1 */
            public static final double SDSMK4_L4 = (5.14 / 1.0);

            /* SDS MK4i */
            /** SDS MK4i - 8.14 : 1 */
            public static final double SDSMK4i_L1 = (8.14 / 1.0);
            /** SDS MK4i - 6.75 : 1 */
            public static final double SDSMK4i_L2 = (6.75 / 1.0);
            /** SDS MK4i - 6.12 : 1 */
            public static final double SDSMK4i_L3 = (6.12 / 1.0);
        }
    }

    public static class SwerveModuleConstants {
        public final int driveMotorID;
        public final int angleMotorID;
        public final int cancoderID;
        public final Rotation2d angleOffset;

        /**
         * Swerve Module Constants to be used when creating swerve modules.
         * @param driveMotorID
         * @param angleMotorID
         * @param canCoderID
         * @param angleOffset
         */
        public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset) {
            this.driveMotorID = driveMotorID;
            this.angleMotorID = angleMotorID;
            this.cancoderID = canCoderID;
            this.angleOffset = angleOffset;
        }
    }

    public class CTREModuleState {

        /**
         * Minimize the change in heading the desired swerve module state would require by potentially
         * reversing the direction the wheel spins. Customized from WPILib's version to include placing
         * in appropriate scope for CTRE onboard control.
         *
         * @param desiredState The desired state.
         * @param currentAngle The current module angle.
         */
        public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
            double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
            double targetSpeed = desiredState.speedMetersPerSecond;
            double delta = targetAngle - currentAngle.getDegrees();
            if (Math.abs(delta) > 90){
                targetSpeed = -targetSpeed;
                targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
            }
            return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
        }

        /**
         * @param scopeReference Current Angle
         * @param newAngle Target Angle
         * @return Closest angle within scope
         */
        private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
            double lowerBound;
            double upperBound;
            double lowerOffset = scopeReference % 360;
            if (lowerOffset >= 0) {
                lowerBound = scopeReference - lowerOffset;
                upperBound = scopeReference + (360 - lowerOffset);
            } else {
                upperBound = scopeReference - lowerOffset;
                lowerBound = scopeReference - (360 + lowerOffset);
            }
            while (newAngle < lowerBound) {
                newAngle += 360;
            }
            while (newAngle > upperBound) {
                newAngle -= 360;
            }
            if (newAngle - scopeReference > 180) {
                newAngle -= 360;
            } else if (newAngle - scopeReference < -180) {
                newAngle += 360;
            }
            return newAngle;
        }
    }
    public final class CTREConfigs {
        public TalonFXConfiguration swerveAngleFXConfig;
        public TalonFXConfiguration swerveDriveFXConfig;
        public CANCoderConfiguration swerveCanCoderConfig;

        public CTREConfigs(){
            swerveAngleFXConfig = new TalonFXConfiguration();
            swerveDriveFXConfig = new TalonFXConfiguration();
            swerveCanCoderConfig = new CANCoderConfiguration();

            /* Swerve Angle Motor Configurations */
            SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
                    Constants.Drivetrain.angleEnableCurrentLimit,
                    Constants.Drivetrain.angleContinuousCurrentLimit,
                    Constants.Drivetrain.anglePeakCurrentLimit,
                    Constants.Drivetrain.anglePeakCurrentDuration);

            swerveAngleFXConfig.slot0.kP = Constants.Drivetrain.angleKP;
            swerveAngleFXConfig.slot0.kI = Constants.Drivetrain.angleKI;
            swerveAngleFXConfig.slot0.kD = Constants.Drivetrain.angleKD;
            swerveAngleFXConfig.slot0.kF = Constants.Drivetrain.angleKF;
            swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

            /* Swerve Drive Motor Configuration */
            SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
                    Constants.Drivetrain.driveEnableCurrentLimit,
                    Constants.Drivetrain.driveContinuousCurrentLimit,
                    Constants.Drivetrain.drivePeakCurrentLimit,
                    Constants.Drivetrain.drivePeakCurrentDuration);

            swerveDriveFXConfig.slot0.kP = Constants.Drivetrain.driveKP;
            swerveDriveFXConfig.slot0.kI = Constants.Drivetrain.driveKI;
            swerveDriveFXConfig.slot0.kD = Constants.Drivetrain.driveKD;
            swerveDriveFXConfig.slot0.kF = Constants.Drivetrain.driveKF;
            swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
            swerveDriveFXConfig.openloopRamp = Constants.Drivetrain.openLoopRamp;
            swerveDriveFXConfig.closedloopRamp = Constants.Drivetrain.closedLoopRamp;

            /* Swerve CANCoder Configuration */
            swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            swerveCanCoderConfig.sensorDirection = Constants.Drivetrain.canCoderInvert;
            swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
            swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        }
    }
}