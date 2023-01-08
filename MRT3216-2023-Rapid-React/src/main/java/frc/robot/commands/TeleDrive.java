package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * A command to drive the robot with joystick input (passed in as
 * {@link DoubleSupplier}s). Written explicitly for pedagogical purposes -
 * actual code should inline a command this simple with
 * {@link edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class TeleDrive extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private DoubleSupplier translationXSupplier;
    private DoubleSupplier translationYSupplier;
    private DoubleSupplier rotationSupplier;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem     The drive subsystem this command wil run on.
     * @param xSpeed        The drive speed in the x direction
     * @param ySpeed        The drive speed int the y direction
     * @param rot           The rotation of the robot
     * @param fieldRelative Whether control is field relative
     */
    public TeleDrive(final SwerveSubsystem swerveSubsystem, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier, boolean fieldRelative) {
        this.swerveSubsystem = swerveSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(this.translationXSupplier.getAsDouble(),
                this.translationYSupplier.getAsDouble(), this.rotationSupplier.getAsDouble(),
                this.swerveSubsystem.getGyroscopeRotation()));
    }
}