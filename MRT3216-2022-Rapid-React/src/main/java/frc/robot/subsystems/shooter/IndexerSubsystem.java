package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Configurations;
import frc.robot.settings.Constants.Shooter.Indexer;
import frc.robot.settings.RobotMap.ROBOT.SHOOTER;

public class IndexerSubsystem extends SubsystemBase {
    private static IndexerSubsystem instance;
    private TalonFX indexerMotor;
    private double shootingVelocityUnitsPer100ms;
    private double indexingVelocityUnitsPer100ms;

    public IndexerSubsystem() {
        this.indexerMotor = new TalonFX(SHOOTER.INDEXER_MOTOR);
        this.indexerMotor.configAllSettings(Configurations.getInstance().getFlywheelMotorConfiguration());

        this.indexerMotor.setNeutralMode(NeutralMode.Brake);
        this.indexerMotor.setInverted(Indexer.INDEXER_MOTOR_INVERTED);
        this.indexerMotor.enableVoltageCompensation(true);

        this.shootingVelocityUnitsPer100ms = Indexer.shootingRPM * 2048.0 / 600.0;
        this.shootingVelocityUnitsPer100ms = Indexer.indexingRPM * 2048.0 / 600.0;
    }

    public void runIndexer(boolean forward) {
        if (forward) {
            indexerMotor.set(TalonFXControlMode.Velocity, this.shootingVelocityUnitsPer100ms);
        } else if (!forward) {
            indexerMotor.set(TalonFXControlMode.PercentOutput, -1 * 0.4);
        }
    }

    public void indexBall() {
        indexerMotor.set(TalonFXControlMode.Velocity, this.indexingVelocityUnitsPer100ms);
    }

    public void stopIndexer() {
        if (indexerMotor != null) {
            indexerMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        }
    }

    public void setShootingRPM(double rPM) {
        this.shootingVelocityUnitsPer100ms = rPM * 2048.0 / 600.0;

    }

    public void setIndexingRPM(double rPM) {
        this.indexingVelocityUnitsPer100ms = rPM * 2048.0 / 600.0;
    }

    public static IndexerSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new IndexerSubsystem();
        }
        return instance;
    }
}
