package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Shooter.Hopper;
import frc.robot.settings.RobotMap.ROBOT.SHOOTER;

public class HopperSubsystem extends SubsystemBase {
    private static HopperSubsystem instance;
    private TalonFX hopperMotor;
    private double percentOutput;

    private HopperSubsystem() {
        this.hopperMotor = new TalonFX(SHOOTER.HOPPER_MOTOR);
        this.hopperMotor.configFactoryDefault();
        this.hopperMotor.setInverted(Hopper.HOPPER_MOTOR_INVERTED);
        this.hopperMotor.setNeutralMode(NeutralMode.Coast);
        this.hopperMotor.enableVoltageCompensation(true);
        
        this.percentOutput = Hopper.kHopperSpeed;
    }

    public void runHopper(boolean forward) {
        if (forward) {
            hopperMotor.set(TalonFXControlMode.PercentOutput, this.percentOutput);
        } else if (!forward) {
            hopperMotor.set(TalonFXControlMode.PercentOutput, -1 * this.percentOutput);
        }
    }

    public void stopHopper() {
        if (hopperMotor != null) {
            hopperMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        }
    }

    public void setPercentOutput(double output) {
        this.percentOutput = output;
    }

    public static HopperSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new HopperSubsystem();
        }
        return instance;
    }
}
