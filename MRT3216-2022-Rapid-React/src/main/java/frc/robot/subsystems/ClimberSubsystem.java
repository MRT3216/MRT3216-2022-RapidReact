package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Climber;
import frc.robot.settings.RobotMap.ROBOT.CLIMBER;

public class ClimberSubsystem extends SubsystemBase {
    private static ClimberSubsystem instance;
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final DigitalInput rightSwitch;
    private final DigitalInput leftSwitch;

    private boolean inverted;
    private boolean tethered;

    private ClimberSubsystem() {
        leftSwitch = new DigitalInput(CLIMBER.LEFT_SWITCH);
        rightSwitch = new DigitalInput(CLIMBER.RIGHT_SWITCH);

        leftMotor = new CANSparkMax(CLIMBER.LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightMotor = new CANSparkMax(CLIMBER.RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        
        leftMotor.setInverted(CLIMBER.LEFT_MOTOR_INVERTED);
        rightMotor.setInverted(CLIMBER.RIGHT_MOTOR_INVERTED);

        leftMotor.enableVoltageCompensation(Climber.kVoltageCompSaturation);
        rightMotor.enableVoltageCompensation(Climber.kVoltageCompSaturation);

        leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.stop();
    }

    public void runMotors(double speed) {
        rightMotor.set(speed);
        leftMotor.set(speed);
    }

    public void runLeftMotor(double speed) {
        if (!this.inverted)
            leftMotor.set(speed);
        else
            leftMotor.set(-speed);
    }

    public void runRightMotor(double speed) {
        if (!this.inverted)
            rightMotor.set(speed);
        else
            rightMotor.set(-speed);
    }

    public boolean isInverted() {
        return this.inverted;
    }

    public void invert() {
        this.inverted = !this.inverted;
    }

    public void tethered(boolean tethered) {
        this.tethered = tethered;
    }

    public boolean isTethered() {
        return this.tethered;
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public static ClimberSubsystem getInstance() {
        if (instance == null) {
            instance = new ClimberSubsystem();
        }
        return instance;
    }
}