package frc.robot.commands.auto.autoProcedures;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.AutoAimAndShoot;
import frc.robot.commands.auto.GoFetch;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.HopperSubsystem;
import frc.robot.subsystems.shooter.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class FiveBall extends SequentialCommandGroup {

    public FiveBall(SwerveSubsystem swerveSystem, IndexerSubsystem indexerSystem,
            ColorSensorSubsystem colorSensorSystem,
            HopperSubsystem hopperSystem, IntakeSubsystem intakeSystem, ShooterSubsystem shooterSystem,
            LimelightSubsystem limelightSystem, HoodSubsystem hoodSystem, double startDelay) {
        super(
                new GoFetch(swerveSystem, indexerSystem, hopperSystem, colorSensorSystem, intakeSystem,
                        shooterSystem, "5Ball1.1"),
                new AutoAimAndShoot(shooterSystem, indexerSystem, hopperSystem, colorSensorSystem,
                        limelightSystem,
                        swerveSystem, hoodSystem),
                new AutoAimAndShoot(shooterSystem, indexerSystem, hopperSystem, colorSensorSystem,
                        limelightSystem,
                        swerveSystem, hoodSystem),
                new GoFetch(swerveSystem, indexerSystem, hopperSystem, colorSensorSystem, intakeSystem,
                        shooterSystem, "5Ball1.2"),
                new AutoAimAndShoot(shooterSystem, indexerSystem, hopperSystem, colorSensorSystem,
                        limelightSystem,
                        swerveSystem, hoodSystem),
                new AutoAimAndShoot(shooterSystem, indexerSystem, hopperSystem, colorSensorSystem,
                        limelightSystem,
                        swerveSystem, hoodSystem),
                new GoFetch(swerveSystem, indexerSystem, hopperSystem, colorSensorSystem, intakeSystem,
                        shooterSystem, "5Ball1.3"));
    }
}