package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Configurations;
import frc.robot.settings.Constants.Shooter.Flywheel;
import frc.robot.settings.RobotMap.ROBOT.SHOOTER;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX flywheelMotor;

    public ShooterSubsystem() {
        flywheelMotor = new TalonFX(SHOOTER.FLYWHEEL_MOTOR);
        flywheelMotor.configAllSettings(Configurations.getInstance().getFlywheelMotorConfiguration());

        flywheelMotor.setInverted(Flywheel.LEFT_FLYWHEEL_MOTOR_INVERTED);
        flywheelMotor.setNeutralMode(NeutralMode.Coast);
        flywheelMotor.enableVoltageCompensation(true);
        flywheelMotor.set(TalonFXControlMode.PercentOutput, 0);

        /* Set status frame periods to ensure we don't have stale data */
        flywheelMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Flywheel.kTimeoutMs);
        flywheelMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Flywheel.kTimeoutMs);
        flywheelMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Flywheel.kTimeoutMs);

        this.zeroSensors();
    }

    @Override
    public void periodic() {
    }

    public void spinToSpeed() {
        // Velocity closed loop without feed forward (not sure if this is enough)
        flywheelMotor.set(TalonFXControlMode.Velocity, Flywheel.targetSpeed);
    }

    public double getRPM() {
        return flywheelMotor.getSelectedSensorVelocity();
    }

    public boolean isReadyToShoot() {
        return getRPM() > Flywheel.acceptableSpeed;
    }

    public void stopShooter() {
        if (flywheelMotor != null) {
            flywheelMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        }
    }

    /* Zero all sensors on Talons */
    void zeroSensors() {
        flywheelMotor.getSensorCollection().setIntegratedSensorPosition(0, Flywheel.kTimeoutMs);
    }
}