package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Climber;
import frc.robot.settings.RobotMap.ROBOT.CLIMBER;

public class ClimberSubsystem extends SubsystemBase {
    private static ClimberSubsystem instance;
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private DigitalInput rightSwitch;
    private DigitalInput leftSwitch;

    private ClimberSubsystem() {
        leftSwitch = new DigitalInput(CLIMBER.LEFT_SWITCH);
        rightSwitch = new DigitalInput(CLIMBER.RIGHT_SWITCH);

        leftMotor = new CANSparkMax(CLIMBER.LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightMotor = new CANSparkMax(CLIMBER.RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftMotor.setInverted(CLIMBER.LEFT_MOTOR_INVERTED);
        rightMotor.setInverted(CLIMBER.RIGHT_MOTOR_INVERTED);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.enableVoltageCompensation(Climber.kVoltageCompSaturation);
        rightMotor.enableVoltageCompensation(Climber.kVoltageCompSaturation);

        leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.stop();
    }

    public void runMotors(double speed) {
        if (!rightSwitch.get() || speed >= 0) {
            rightMotor.set(speed);
        }

        if (!leftSwitch.get() || speed >= 0) {
            leftMotor.set(speed);
        }
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