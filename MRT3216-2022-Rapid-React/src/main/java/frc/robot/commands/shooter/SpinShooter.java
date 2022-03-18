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
    private final BooleanSupplier eject;

    public SpinShooter(final ShooterSubsystem shooterSubsystem, BooleanSupplier isForward, BooleanSupplier eject) {
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.isForward = isForward;
        this.eject = eject;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (this.eject.getAsBoolean()) {
            shooterSubsystem.eject();
        } else {
            shooterSubsystem.spinToSpeed(isForward.getAsBoolean());
        }
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