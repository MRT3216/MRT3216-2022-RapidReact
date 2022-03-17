package frc.robot.settings;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.settings.Constants.Intake;
import frc.robot.settings.Constants.Shooter.Flywheel;
import frc.robot.settings.Constants.Shooter.Hood;
import frc.robot.settings.Constants.Shooter.Hopper;
import frc.robot.settings.Constants.Shooter.Indexer;

public class Configurations {
    private TalonFXConfiguration intakeMotorConfiguration;
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

    public TalonFXConfiguration getSwerveDriveMotorConfiguration() {
        if (swerveDriveMotorConfiguration == null) {
            swerveDriveMotorConfiguration = new TalonFXConfiguration();
        }

        return swerveDriveMotorConfiguration;
    }

    public TalonFXConfiguration getSwerveAngleMotorConfiguration() {
        if (swerveAngleMotorConfiguration == null) {
            swerveAngleMotorConfiguration = new TalonFXConfiguration();
        }

        return swerveAngleMotorConfiguration;
    }
}