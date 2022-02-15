package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.settings.RobotMap;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final DoubleSolenoid intakePiston;

    public IntakeSubsystem() {
        motor = new CANSparkMax(RobotMap.ROBOT.INTAKE.INTAKE_MOTOR, Constants.kBrusheless);
        this.intakePiston = new DoubleSolenoid(null,RobotMap.ROBOT.PNEUMATICS.INTAKE_FORWARD, RobotMap.ROBOT.PNEUMATICS.INTAKE_REVERSE);
        motor.setInverted(false);
    }

    public void setOn(final boolean forward) {
        if (forward) {
            motor.set(Constants.Intake.kForwardIntakeSpeed);
        }
        else {
            motor.set(-1 * Constants.Intake.kReverseIntakeSpeed);
        }
    }

    public void setOff() {
        motor.set(0);
    }

    public void extendIntake() {
        intakePiston.set(DoubleSolenoid.Value.kForward);
    }

    public void retractIntake() {
        intakePiston.set(DoubleSolenoid.Value.kReverse);
    }
}
