// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class AdjustHood extends CommandBase {
    private final HoodSubsystem hoodSystem;

    /** Creates a new AdjustHood. */
    public AdjustHood(HoodSubsystem hoodSystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(hoodSystem);
        this.hoodSystem = hoodSystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double launchAngle = hoodSystem.getProjectileLaunchAngle();
        hoodSystem.setHoodAngle(launchAngle);
        hoodSystem.enable();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double launchAngle = hoodSystem.getProjectileLaunchAngle();
        hoodSystem.setHoodAngle(launchAngle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return hoodSystem.atGoal();
    }
}
