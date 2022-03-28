package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.shooter.*;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.HopperSubsystem;
import frc.robot.subsystems.shooter.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoAimAndShoot extends CommandBase {
    public AutoAimAndShoot (ShooterSubsystem shooterSystem, IndexerSubsystem indexerSystem,
                            HopperSubsystem hopperSystem, ColorSensorSubsystem colorSensorSystem,
                            LimelightSubsystem limelightSystem, SwerveSubsystem swerveSystem,
                            HoodSubsystem hoodSystem) {
        new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new ConditionalCommand(new AutoAimDrivebase(swerveSystem, limelightSystem), new InstantCommand(),
                                limelightSystem::hasTarget),
                        new AdjustHood(hoodSystem)
                ),
                new FireCargo(shooterSystem, indexerSystem, hopperSystem, colorSensorSystem, limelightSystem),
                new RunIndexer(indexerSystem, () -> true,
                        () -> shooterSystem.isReadyToShoot(),
                        () -> colorSensorSystem.isOpponentBall()),
                new SpinShooter(shooterSystem, () -> true, () -> false, limelightSystem::getInitialRPM)
        );
    }
}
