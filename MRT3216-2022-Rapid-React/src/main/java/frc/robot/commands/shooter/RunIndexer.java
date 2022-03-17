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
    private final BooleanSupplier isForward;
    private final BooleanSupplier run;
    private final BooleanSupplier finish;

    public RunIndexer(final IndexerSubsystem indexer, final BooleanSupplier isForward, final BooleanSupplier run,
            final BooleanSupplier finish) {
        this.indexer = indexer;
        this.isForward = isForward;
        this.run = run;
        this.finish = finish;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.run.getAsBoolean()) {
            indexer.runIndexer(isForward.getAsBoolean());
        } else {
            indexer.stopIndexer();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
        indexer.stopIndexer();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.finish.getAsBoolean();
    }
}
