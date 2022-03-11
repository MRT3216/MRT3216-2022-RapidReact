package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.settings.RobotMap.ROBOT.SHOOTER;

public class HopperSubsystem extends SubsystemBase {
    private TalonFX hopperMotor;

    public HopperSubsystem() {
        this.hopperMotor = new TalonFX(SHOOTER.HOPPER_MOTOR);
        this.hopperMotor.setInverted(Constants.Shooter.Hopper.HOPPER_MOTOR_INVERTED);
    }

    public void runHopper(boolean forward) {
        if (forward) {
            hopperMotor.set(TalonFXControlMode.PercentOutput, Constants.Shooter.Hopper.kHopperSpeed);
        } else if (!forward) {
            hopperMotor.set(TalonFXControlMode.PercentOutput, -1 * Constants.Shooter.Hopper.kHopperSpeed);
        }
    }

    public void stopHopper() {
        if (hopperMotor != null) {
            hopperMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        }
    }
}
