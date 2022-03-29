package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.AdjustHood;
import frc.robot.commands.shooter.RunHopper;
import frc.robot.commands.shooter.RunIndexer;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.HopperSubsystem;
import frc.robot.subsystems.shooter.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoAimAndShoot extends SequentialCommandGroup {
    public AutoAimAndShoot(ShooterSubsystem shooterSystem, IndexerSubsystem indexerSystem,
            HopperSubsystem hopperSystem, ColorSensorSubsystem colorSensorSystem,
            LimelightSubsystem limelightSystem, SwerveSubsystem swerveSystem,
            HoodSubsystem hoodSystem) {

        super(
                new ParallelCommandGroup(
                        new ConditionalCommand(
                                new AutoAimDrivebase(swerveSystem, limelightSystem),
                                new InstantCommand(),
                                limelightSystem::hasTarget),
                        new AdjustHood(hoodSystem)),
                new ParallelDeadlineGroup(
                        new AutoSpinShooter(shooterSystem, () -> true, () -> false,
                                limelightSystem::getInitialRPM),
                        new RunHopper(hopperSystem, () -> true),
                        new RunIndexer(indexerSystem, () -> true,
                                shooterSystem::isReadyToShoot,
                                colorSensorSystem::isOpponentBall)));
    }
}
