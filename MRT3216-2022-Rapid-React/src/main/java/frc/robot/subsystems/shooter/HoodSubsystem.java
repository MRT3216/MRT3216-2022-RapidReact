package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.settings.Constants.Shooter.Hood;
import frc.robot.settings.RobotMap;

/** A robot hood subsystem that moves with a motion profile. */
public class HoodSubsystem extends ProfiledPIDSubsystem {
    private static HoodSubsystem instance;
    private final CANSparkMax motor = new CANSparkMax(RobotMap.ROBOT.SHOOTER.HOOD_MOTOR, MotorType.kBrushless);

    private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(RobotMap.ROBOT.SHOOTER.HOOD_ENCODER_PWM_PORT);

    private final ArmFeedforward m_feedforward = new ArmFeedforward(
            Hood.kS, Hood.kG,
            Hood.kV, Hood.kA);

    /** Create a new HoodSubsystem. */
    private HoodSubsystem() {
        super(
                new ProfiledPIDController(
                        Hood.kP,
                        Hood.kI,
                        Hood.kD,
                        new TrapezoidProfile.Constraints(
                                Hood.kMaxVelocity,
                                Hood.kMaxAcceleration)),
                0);

        motor.restoreFactoryDefaults();
        motor.enableVoltageCompensation(Hood.kVoltageCompSaturation);
        m_encoder.setDistancePerRotation(2 * Math.PI);
        motor.setIdleMode(IdleMode.kCoast);
        // Start arm at rest in neutral position
        setGoal(Hood.kArmOffsetRads);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        motor.setVoltage(output + feedforward);
    }

    @Override
    public double getMeasurement() {
        return m_encoder.getAbsolutePosition();
        // return m_encoder.getDistance() + Hood.kArmOffsetRads;
    }

    public static HoodSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new HoodSubsystem();
        }
        return instance;
    }
}