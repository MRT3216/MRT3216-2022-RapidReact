package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
    private final DoubleSupplier targetRPM;

    public SpinShooter(final ShooterSubsystem shooterSubsystem, BooleanSupplier isForward, BooleanSupplier eject) {
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.isForward = isForward;
        this.eject = eject;
        this.targetRPM = null;
    }

    public SpinShooter(final ShooterSubsystem shooterSubsystem, BooleanSupplier isForward, BooleanSupplier eject,
            DoubleSupplier targetRPM) {
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.isForward = isForward;
        this.eject = eject;
        this.targetRPM = targetRPM;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (this.targetRPM != null) {
            this.shooterSubsystem.setShootingRPM(targetRPM.getAsDouble());
        }
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