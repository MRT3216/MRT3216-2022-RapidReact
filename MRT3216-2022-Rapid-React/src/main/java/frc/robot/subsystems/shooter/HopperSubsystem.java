package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.settings.RobotMap;

public class HopperSubsystem extends SubsystemBase {
    private final CANSparkMax hopperMotor;

    public HopperSubsystem () {
        this.hopperMotor = new CANSparkMax(RobotMap.ROBOT.SHOOTER.HOPPER_MOTOR, Constants.kBrusheless);
        this.hopperMotor.setInverted(Constants.Shooter.Hopper.HOPPER_MOTOR_INVERTED);
    }

    public void runHopper(boolean forward, boolean on) {
        if (on && forward) {
            hopperMotor.set(Constants.Intake.kForwardIntakeSpeed);
        }
        else if (on && forward) {
            hopperMotor.set(-1 *Constants.Intake.kForwardIntakeSpeed);
        }
        else {
            stopHopper();
        }
    }

    public void stopHopper() {
        hopperMotor.stopMotor();
    }
}
