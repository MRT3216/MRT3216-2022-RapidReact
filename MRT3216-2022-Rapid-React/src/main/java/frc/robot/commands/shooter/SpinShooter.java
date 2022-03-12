package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Spins up the shooter anticipating to shoot a specific distance until the
 * command is interrupted.
 */
public class SpinShooter extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;
    private final BooleanSupplier isForward;

    public SpinShooter(final ShooterSubsystem shooterSubsystem, BooleanSupplier isForward) {
        this.shooterSubsystem = shooterSubsystem;
        this.isForward = isForward;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooterSubsystem.spinToSpeed(isForward.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(final boolean interrupted) {
        shooterSubsystem.stopShooter();
    }
}