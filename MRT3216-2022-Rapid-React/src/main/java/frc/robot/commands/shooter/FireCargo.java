/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auto.AutoSpinShooter;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.shooter.HopperSubsystem;
import frc.robot.subsystems.shooter.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FireCargo extends ParallelCommandGroup {

    /**
     * Creates a new FirePowerCells.
     */
    public FireCargo(ShooterSubsystem shooterSystem, IndexerSubsystem indexer, HopperSubsystem hopper,
            ColorSensorSubsystem colorSensor, LimelightSubsystem limelightSystem) {
        super(
                new RunHopper(hopper, () -> true),
                new RunIndexer(indexer, () -> true,
                        () -> shooterSystem.isReadyToShoot(),
                        () -> colorSensor.isOpponentBall()),
                new AutoSpinShooter(shooterSystem, () -> true, () -> false, limelightSystem::getInitialRPM));
    }
}