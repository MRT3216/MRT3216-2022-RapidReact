/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.IndexerSubsystem;

public class RunIndexer extends CommandBase {
    private final IndexerSubsystem indexer;
    private final BooleanSupplier run;

    public RunIndexer(final IndexerSubsystem indexer, final BooleanSupplier run) {
        this.indexer = indexer;
        this.run = run;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        indexer.activateIndexer(run.getAsBoolean());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
        indexer.activateIndexer(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
