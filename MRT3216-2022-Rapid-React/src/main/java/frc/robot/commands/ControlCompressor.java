/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CompressorSubsystem;

public class ControlCompressor extends CommandBase {
    private final CompressorSubsystem compressorSystem;
    private final boolean on;

    public ControlCompressor(final CompressorSubsystem compressor, final boolean on) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.compressorSystem = compressor;
        this.on = on;
        addRequirements(this.compressorSystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (on) {
            this.compressorSystem.start();
        } else {
            this.compressorSystem.stop();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}