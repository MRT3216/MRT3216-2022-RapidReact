/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.HopperSubsystem;
import frc.robot.subsystems.shooter.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AimAndFireCargo extends SequentialCommandGroup {

    /**
     * Creates a new FirePowerCells.
     */
    public AimAndFireCargo(ShooterSubsystem shooterSystem, IndexerSubsystem indexerSystem,
            HopperSubsystem hopperSystem, ColorSensorSubsystem colorSensorSystem,
            LimelightSubsystem limelightSystem, SwerveSubsystem swerveSystem,
            HoodSubsystem hoodSystem) {
        super(
                new ParallelCommandGroup(
                        new AdjustHood(hoodSystem),
                        new ConditionalCommand(
                                new AimDrivebase(swerveSystem, limelightSystem),
                                new InstantCommand(),
                                limelightSystem::hasTarget)),
                new FireCargo(shooterSystem, indexerSystem, hopperSystem, colorSensorSystem,
                        limelightSystem));
    }
}