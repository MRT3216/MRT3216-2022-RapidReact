package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
<<<<<<< Updated upstream

=======
import edu.wpi.first.wpilibj.DigitalInput;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Climber;
import frc.robot.settings.RobotMap.ROBOT.CLIMBER;

public class ClimberSubsystem extends SubsystemBase {
    private static ClimberSubsystem instance;
    private static CANSparkMax leftMotor;
    private static CANSparkMax rightMotor;

<<<<<<< Updated upstream
=======
    private static DigitalInput leftSwitch;
    private static DigitalInput rightSwitch;
    private static DigitalInput leftUpperSwitch;
    private static DigitalInput rightUpperSwitch;

    private boolean isForward;
    private boolean stater;


>>>>>>> Stashed changes
    private ClimberSubsystem() {
        leftSwitch = new DigitalInput(CLIMBER.LEFT_SWITCH);
        rightSwitch = new DigitalInput(CLIMBER.RIGHT_SWITCH);

        leftMotor = new CANSparkMax(CLIMBER.LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightMotor = new CANSparkMax(CLIMBER.RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.enableVoltageCompensation(Climber.kVoltageCompSaturation);
        rightMotor.enableVoltageCompensation(Climber.kVoltageCompSaturation);

        leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        isForward = true;
        stater = false;
    }

<<<<<<< Updated upstream
    public void runForward() {

=======
    public void runForward(double speed) {
        
>>>>>>> Stashed changes
    }

    public static ClimberSubsystem getInstance() {
        if (instance == null) {
            instance = new ClimberSubsystem();
        }
        return instance;
    }
}