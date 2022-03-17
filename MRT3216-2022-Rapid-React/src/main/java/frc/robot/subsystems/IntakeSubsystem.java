package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Intake;
import frc.robot.settings.RobotMap.ROBOT.INTAKE;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;
    private final CANSparkMax motor;
    private double percentOutputForward;
    private double percentOutputReverse;

    /**
     * Creates a new Intake.
     */
    private IntakeSubsystem() {
        this.motor = new CANSparkMax(INTAKE.INTAKE_MOTOR, MotorType.kBrushless);
        this.motor.restoreFactoryDefaults();
        this.motor.enableVoltageCompensation(Intake.kVoltageCompSaturation);
        this.percentOutputForward = Intake.kForwardIntakeSpeed;
        this.percentOutputReverse = Intake.kReverseIntakeSpeed;
        this.motor.setInverted(false);
        this.stopIntake();
    }

    public void runIntake(final boolean forward) {
        if (forward) {
            motor.set(this.percentOutputForward);
        } else {
            motor.set(-1 * this.percentOutputReverse);
        }
    }

    public void stopIntake() {
        motor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setForwardPercentOutput(double output) {
        this.percentOutputForward = output;
    }

    public void setReversePercentOutput(double output) {
        this.percentOutputForward = output;
    }

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new IntakeSubsystem();
        }
        return instance;
    }
}
