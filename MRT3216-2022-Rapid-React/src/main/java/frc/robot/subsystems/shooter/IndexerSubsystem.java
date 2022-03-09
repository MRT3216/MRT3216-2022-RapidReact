package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.settings.RobotMap;

public class IndexerSubsystem extends SubsystemBase {
    private final CANSparkMax motor;

    public IndexerSubsystem() {
        motor = new CANSparkMax(RobotMap.ROBOT.INTAKE.INTAKE_MOTOR, Constants.kBrusheless);
        motor.setInverted(false);
    }

    public void setOn(final boolean forward) {
        if (forward) {
            motor.set(Constants.Intake.kForwardIntakeSpeed);
        } else {
            motor.set(-1 * Constants.Intake.kReverseIntakeSpeed);
        }
    }

    public void setOff() {
        motor.set(0);
    }
}
