package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Shooter.Hopper;
import frc.robot.settings.RobotMap.ROBOT.SHOOTER;

public class HopperSubsystem extends SubsystemBase {
    private TalonFX hopperMotor;
    private static HopperSubsystem instance;
    public HopperSubsystem() {
        this.hopperMotor = new TalonFX(SHOOTER.HOPPER_MOTOR);
        this.hopperMotor.setInverted(Hopper.HOPPER_MOTOR_INVERTED);
    }

    public void runHopper(boolean forward) {
        if (forward) {
            hopperMotor.set(TalonFXControlMode.PercentOutput, Hopper.kHopperSpeed);
        } else if (!forward) {
            hopperMotor.set(TalonFXControlMode.PercentOutput, -1 * Hopper.kHopperSpeed);
        }
    }

    public void stopHopper() {
        if (hopperMotor != null) {
            hopperMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        }
    }

    public static HopperSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new HopperSubsystem();
        }
        return instance;
    }
}
