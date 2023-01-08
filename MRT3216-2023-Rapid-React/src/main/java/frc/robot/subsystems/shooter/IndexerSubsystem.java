package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterStateMachine;
import frc.robot.settings.Configurations;
import frc.robot.settings.Constants.Shooter.Indexer;
import frc.robot.settings.RobotMap.ROBOT.SHOOTER;
import frc.robot.settings.Utilities;

public class IndexerSubsystem extends SubsystemBase {
    private static IndexerSubsystem instance;
    private TalonFX indexerMotor;
    private double shootingVelocityUnitsPer100ms;
    private double indexingVelocityUnitsPer100ms;

    private IndexerSubsystem() {
        this.indexerMotor = new TalonFX(SHOOTER.INDEXER_MOTOR);
        this.indexerMotor.configFactoryDefault();
        this.indexerMotor.configAllSettings(Configurations.getInstance().getIndexerMotorConfiguration());

        this.indexerMotor.setNeutralMode(NeutralMode.Brake);
        this.indexerMotor.setInverted(Indexer.INDEXER_MOTOR_INVERTED);
        this.indexerMotor.enableVoltageCompensation(true);
        this.stopIndexer();

        this.shootingVelocityUnitsPer100ms = Utilities.convertRPMsToUnitsPer100ms(Indexer.shootingRPM, Indexer.kSensorUnitsPerRotation);
        this.indexingVelocityUnitsPer100ms = Utilities.convertRPMsToUnitsPer100ms(Indexer.indexingRPM, Indexer.kSensorUnitsPerRotation);
    }

    public void runIndexer(boolean forward) {
        if (forward) {
            indexerMotor.set(TalonFXControlMode.Velocity, this.shootingVelocityUnitsPer100ms);
        } else if (!forward) {
            indexerMotor.set(TalonFXControlMode.PercentOutput, -1 * 0.4);
            ShooterStateMachine.getInstance().reverseEject();
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

    public double getRPM() {
        return Utilities.convertUnitsPer100msToRPM(indexerMotor.getSelectedSensorVelocity(),
                Indexer.kSensorUnitsPerRotation);
    }

    public void setShootingRPM(double rpm) {
        this.shootingVelocityUnitsPer100ms = Utilities.convertRPMsToUnitsPer100ms(rpm, Indexer.kSensorUnitsPerRotation);
    }

    public void setIndexingRPM(double rpm) {
        this.indexingVelocityUnitsPer100ms = Utilities.convertRPMsToUnitsPer100ms(rpm, Indexer.kSensorUnitsPerRotation);
    }

    public void setPValue(double p) {
        this.indexerMotor.config_kP(Indexer.kSlotIdx, p);
    }

    public void setIValue(double i) {
        this.indexerMotor.config_kP(Indexer.kSlotIdx, i);
    }

    public void setDValue(double d) {
        this.indexerMotor.config_kP(Indexer.kSlotIdx, d);
    }

    public void setFValue(double f) {
        this.indexerMotor.config_kF(Indexer.kSlotIdx, f);
    }

    public static IndexerSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new IndexerSubsystem();
        }
        return instance;
    }
}
