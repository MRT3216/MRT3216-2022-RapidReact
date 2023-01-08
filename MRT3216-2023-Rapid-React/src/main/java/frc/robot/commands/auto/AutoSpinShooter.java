package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShooterStateMachine;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Spins up the shooter anticipating to shoot a specific distance until the
 * command is interrupted.
 */
public class AutoSpinShooter extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;
    private final BooleanSupplier isForward;
    private final BooleanSupplier eject;
    private final DoubleSupplier targetRPM;
    private ShooterStateMachine shooterStateMachine;
    private int numBalls = 0;

    public AutoSpinShooter(final ShooterSubsystem shooterSubsystem, BooleanSupplier isForward, BooleanSupplier eject,
            int numBalls) {
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.shooterStateMachine = ShooterStateMachine.getInstance();
        this.isForward = isForward;
        this.eject = eject;
        this.targetRPM = null;
        this.numBalls = numBalls;
    }

    public AutoSpinShooter(final ShooterSubsystem shooterSubsystem, BooleanSupplier isForward, BooleanSupplier eject,
            int numBalls, DoubleSupplier targetRPM) {
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.shooterStateMachine = ShooterStateMachine.getInstance();
        this.isForward = isForward;
        this.eject = eject;
        this.numBalls = numBalls;
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
        if (this.shooterStateMachine.ballsShot() == this.numBalls) {
            this.shooterStateMachine.resetShot();
            return true;
        }
        return false;
    }

    @Override
    public void end(final boolean interrupted) {
        shooterSubsystem.stopShooter();
    }
}