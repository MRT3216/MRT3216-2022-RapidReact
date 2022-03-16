package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Climber;
import frc.robot.settings.RobotMap.ROBOT.CLIMBER;

public class ClimberSubsystem extends SubsystemBase {
    private static ClimberSubsystem instance;
    private static CANSparkMax leftMotor;
    private static CANSparkMax rightMotor;

    private ClimberSubsystem() {
        leftMotor = new CANSparkMax(CLIMBER.LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightMotor = new CANSparkMax(CLIMBER.RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.enableVoltageCompensation(Climber.kVoltageCompSaturation);
        rightMotor.enableVoltageCompensation(Climber.kVoltageCompSaturation);

        leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void runForward() {

    }

    public static ClimberSubsystem getInstance() {
        if (instance == null) {
            instance = new ClimberSubsystem();
        }
        return instance;
    }
}