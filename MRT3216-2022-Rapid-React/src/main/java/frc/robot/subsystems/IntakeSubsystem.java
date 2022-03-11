package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.settings.RobotMap.ROBOT.INTAKE;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax motor;

    /**
     * Creates a new Intake.
     */
    public IntakeSubsystem() {
        motor = new CANSparkMax(INTAKE.INTAKE_MOTOR, Constants.kBrusheless);
        motor.setInverted(false);
    }

    public void runIntake(final boolean forward) {
        if (forward) {
            motor.set(Constants.Intake.kForwardIntakeSpeed);
        } else {
            motor.set(-1 * Constants.Intake.kReverseIntakeSpeed);
        }
    }

    public void stopIntake() {
        motor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
