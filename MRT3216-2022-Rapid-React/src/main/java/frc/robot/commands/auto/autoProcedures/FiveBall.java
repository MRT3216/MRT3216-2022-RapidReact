package frc.robot.commands.auto.autoProcedures;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.auto.AutoAimAndShoot;
import frc.robot.commands.auto.GoFetch;
import frc.robot.commands.shooter.RunIntake;
import frc.robot.settings.Constants;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.HopperSubsystem;
import frc.robot.subsystems.shooter.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class FiveBall extends ParallelCommandGroup {

        public FiveBall(SwerveSubsystem swerveSystem, IndexerSubsystem indexerSystem,
                        ColorSensorSubsystem colorSensorSystem,
                        HopperSubsystem hopperSystem, IntakeSubsystem intakeSystem, ShooterSubsystem shooterSystem,
                        LimelightSubsystem limelightSystem, HoodSubsystem hoodSystem, double startDelay) {
                super(
                        new FunctionalCommand(
                                () -> {
                                        swerveSystem.zeroGyroscope();
                                }, // OnInit: zero gyro
                                () -> {
                                }, // OnExecute: do nothing
                                interrupted -> new InstantCommand(), // OnEnd: stop motors
                                () -> true // IsFinished: never finish
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(Constants.Auto.kStartDelayTime),
                                new GoFetch(swerveSystem, indexerSystem, hopperSystem, colorSensorSystem, intakeSystem,
                                        shooterSystem, "5Ball1.1"),
                                new WaitCommand(Constants.Auto.kDriveToShootDelay),
                                new AutoAimAndShoot(shooterSystem, indexerSystem, hopperSystem, colorSensorSystem, limelightSystem,
                                        swerveSystem, hoodSystem, 2),
                                new GoFetch(swerveSystem, indexerSystem, hopperSystem, colorSensorSystem, intakeSystem,
                                        shooterSystem, "5Ball1.2"),
                                new WaitCommand(Constants.Auto.kDriveToShootDelay),
                                new AutoAimAndShoot(shooterSystem, indexerSystem, hopperSystem, colorSensorSystem, limelightSystem,
                                        swerveSystem, hoodSystem, 2),

                                new GoFetch(swerveSystem, indexerSystem, hopperSystem, colorSensorSystem, intakeSystem,
                                        shooterSystem, "5Ball1.3"),
                                new WaitCommand(Constants.Auto.kDriveToShootDelay),
                                new AutoAimAndShoot(shooterSystem, indexerSystem, hopperSystem, colorSensorSystem, limelightSystem,
                                        swerveSystem, hoodSystem, 1)
                        ),
                        new RunIntake(intakeSystem, () -> true)

                );
        }
}