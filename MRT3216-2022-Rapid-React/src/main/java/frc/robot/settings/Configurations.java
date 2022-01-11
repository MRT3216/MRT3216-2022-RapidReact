package frc.robot.settings;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.settings.Constants.Shooter.Flywheel;

public class Configurations {
    private TalonFXConfiguration hopperMotorConfiguration;
    private TalonFXConfiguration intakeMotorConfiguration;
    private TalonFXConfiguration leftFlywheelMotorConfiguration;
    private TalonFXConfiguration rightFlywheelMotorConfiguration;
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
        }

        return hopperMotorConfiguration;
    }

    public TalonFXConfiguration getIntakeMotorConfiguration() {
        if (intakeMotorConfiguration == null) {
            intakeMotorConfiguration = new TalonFXConfiguration();
        }

        return intakeMotorConfiguration;
    }

    public TalonFXConfiguration getLeftFlywheelMotorConfiguration() {
        if (leftFlywheelMotorConfiguration == null) {
            leftFlywheelMotorConfiguration = new TalonFXConfiguration();
            leftFlywheelMotorConfiguration.voltageCompSaturation = Flywheel.kVoltageCompSaturation;
            leftFlywheelMotorConfiguration.slot2.kF = Flywheel.kShooterGains.kF;
            leftFlywheelMotorConfiguration.slot2.kP = Flywheel.kShooterGains.kP;
            leftFlywheelMotorConfiguration.slot2.kI = Flywheel.kShooterGains.kI;
            leftFlywheelMotorConfiguration.slot2.kD = Flywheel.kShooterGains.kD;
            leftFlywheelMotorConfiguration.slot2.integralZone = Flywheel.kShooterGains.kIzone;
            leftFlywheelMotorConfiguration.slot2.closedLoopPeakOutput = Flywheel.kShooterGains.kPeakOutput;
            leftFlywheelMotorConfiguration.closedloopRamp = Flywheel.RAMP_RATE;
            leftFlywheelMotorConfiguration.neutralDeadband = Flywheel.kNeutralDeadband;
            leftFlywheelMotorConfiguration.slot0.closedLoopPeriod = Flywheel.closedLoopTimeMs;
            leftFlywheelMotorConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor
                    .toFeedbackDevice();
        }

        return leftFlywheelMotorConfiguration;
    }

    public TalonFXConfiguration getRightFlywheelMotorConfiguration() {
        if (rightFlywheelMotorConfiguration == null) {
            rightFlywheelMotorConfiguration = new TalonFXConfiguration();
            rightFlywheelMotorConfiguration.voltageCompSaturation = Flywheel.kVoltageCompSaturation;;
            rightFlywheelMotorConfiguration.slot2.kF = Flywheel.kShooterGains.kF;
            rightFlywheelMotorConfiguration.slot2.kP = Flywheel.kShooterGains.kP;
            rightFlywheelMotorConfiguration.slot2.kI = Flywheel.kShooterGains.kI;
            rightFlywheelMotorConfiguration.slot2.kD = Flywheel.kShooterGains.kD;
            rightFlywheelMotorConfiguration.slot2.integralZone = Flywheel.kShooterGains.kIzone;
            rightFlywheelMotorConfiguration.slot2.closedLoopPeakOutput = Flywheel.kShooterGains.kPeakOutput;
            rightFlywheelMotorConfiguration.closedloopRamp = Flywheel.RAMP_RATE;
            rightFlywheelMotorConfiguration.neutralDeadband = Flywheel.kNeutralDeadband;
            rightFlywheelMotorConfiguration.slot0.closedLoopPeriod = Flywheel.closedLoopTimeMs;
        }

        return rightFlywheelMotorConfiguration;
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