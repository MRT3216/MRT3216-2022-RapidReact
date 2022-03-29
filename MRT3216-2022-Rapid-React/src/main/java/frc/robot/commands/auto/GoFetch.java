package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.shooter.IndexCargo;
import frc.robot.commands.shooter.RunHopper;
import frc.robot.commands.shooter.RunIntake;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.settings.Constants.Auto;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.shooter.HopperSubsystem;
import frc.robot.subsystems.shooter.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class GoFetch extends ParallelDeadlineGroup {
    // my sense of humor is writing a class called GoFetch lol
    public GoFetch(SwerveSubsystem swerveSystem, IndexerSubsystem indexerSystem, HopperSubsystem hopperSystem,
            ColorSensorSubsystem colorSensorSystem, IntakeSubsystem intakeSystem,
            ShooterSubsystem shooterSystem, String pathname) {
        super(
                new DriveHolonomicTrajectory(swerveSystem,
                        PathPlanner.loadPath(pathname, Auto.kMaxFetchVelocity,
                                Auto.kMaxFetchAcc)),
                new IndexCargo(indexerSystem, () -> colorSensorSystem.isAllianceBall()),
                new RunHopper(hopperSystem, () -> true),
                new RunIntake(intakeSystem, () -> true),
                new SpinShooter(shooterSystem, () -> true, () -> true));
    }
}
