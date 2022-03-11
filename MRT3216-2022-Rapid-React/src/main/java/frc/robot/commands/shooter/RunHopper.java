/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.HopperSubsystem;

public class RunHopper extends CommandBase {
    private final HopperSubsystem hopper;
    private final BooleanSupplier run;

    public RunHopper(HopperSubsystem hopper, BooleanSupplier run) {
        this.hopper = hopper;
        this.run = run;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        hopper.activateHopper(run.getAsBoolean());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        hopper.activateHopper(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
