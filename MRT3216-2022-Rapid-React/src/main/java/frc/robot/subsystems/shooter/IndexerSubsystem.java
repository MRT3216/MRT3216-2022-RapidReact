package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Shooter.Indexer;
import frc.robot.settings.RobotMap.ROBOT.SHOOTER;

public class IndexerSubsystem extends SubsystemBase {
    private static IndexerSubsystem instance;
    private TalonFX indexerMotor;
    private double percentOutput;

    public IndexerSubsystem() {
        this.indexerMotor = new TalonFX(SHOOTER.INDEXER_MOTOR);
        this.indexerMotor.setInverted(Indexer.INDEXER_MOTOR_INVERTED);
        this.percentOutput = Indexer.kIndexerSpeed;
    }

    public void runIndexer(boolean forward) {
        if (forward) {
            indexerMotor.set(TalonFXControlMode.PercentOutput, this.percentOutput);
        } else if (!forward) {
            indexerMotor.set(TalonFXControlMode.PercentOutput, -1 * this.percentOutput);
        }
    }

    public void stopIndexer() {
        if (indexerMotor != null) {
            indexerMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        }
    }

    public void setPercentOutput(double output) {
        this.percentOutput = output;
    }

    public static IndexerSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new IndexerSubsystem();
        }
        return instance;
    }
}
