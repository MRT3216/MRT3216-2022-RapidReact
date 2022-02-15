package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.settings.RobotMap;

public class HopperSubsystem extends SubsystemBase {
    private final TalonFX hopperMotor;

    public HopperSubsystem () {
        this.hopperMotor = new TalonFX(RobotMap.ROBOT.SHOOTER.HOPPER_MOTOR);
        this.hopperMotor.setInverted(Constants.Shooter.Hopper.HOPPER_MOTOR_INVERTED);
    }

    public void runHopper(boolean forward, boolean on) {
        if (on && forward) {
            hopperMotor.set(ControlMode.PercentOutput, Constants.Shooter.Hopper.kHopperSpeed);
        }
        else if (on && forward) {
            hopperMotor.set(ControlMode.PercentOutput, -1 * Constants.Shooter.Hopper.kHopperSpeed);
        }
        else {
            stopHopper();
        }
    }

    public void stopHopper() {
        hopperMotor.set(ControlMode.PercentOutput, 0);
    }
}
