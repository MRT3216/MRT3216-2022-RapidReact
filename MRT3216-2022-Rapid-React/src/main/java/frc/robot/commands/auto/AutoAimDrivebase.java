// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.Constants.Auto;
import frc.robot.settings.Constants.Drivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAimDrivebase extends CommandBase {
    private SwerveSubsystem swerveSystem;
    private LimelightSubsystem limelightSystem;
    private ProfiledPIDController controller;
    private boolean neverSawTarget = true;

    /**
     * Creates a new AimDrivebase.
     */
    public AutoAimDrivebase(SwerveSubsystem swerveSystem, LimelightSubsystem limelightSystem) {
        this.swerveSystem = swerveSystem;
        this.limelightSystem = limelightSystem;

        addRequirements(swerveSystem);
    }

    @Override
    public void initialize() {
        this.controller = new ProfiledPIDController(swerveSystem.getThetaGains().kP, swerveSystem.getThetaGains().kI,
                swerveSystem.getThetaGains().kD, // Theta
                // controller
                new TrapezoidProfile.Constraints(
                        Units.radiansToDegrees(Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
                        Units.radiansToDegrees(Drivetrain.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_PER_SECOND))); // Max
                                                                                                                     // angular
        // acceleration
        this.controller.enableContinuousInput(-180, 180);
        controller.setTolerance(Auto.kMaxTurnRateErrorAuto, Auto.kMaxTurnRateErrorAuto);

        this.controller.reset(swerveSystem.getGyroscopeRotation().getDegrees());
        if (limelightSystem.hasTarget()) {
            this.neverSawTarget = false;
            Rotation2d offset2d = Rotation2d.fromDegrees(limelightSystem.getHorizontalOffset());
            Rotation2d r2d = swerveSystem.getGyroscopeRotation().rotateBy(offset2d);
            this.controller.setGoal(r2d.getDegrees());
        } else {
            end(true);
        }
    }

    @Override
    public void execute() {
        if (!neverSawTarget) {
            double omega = this.controller.calculate(swerveSystem.getGyroscopeRotation().getDegrees());
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, Units.degreesToRadians(omega));
            swerveSystem.drive(chassisSpeeds);
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        System.out.println("Ending: " + interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return controller.atGoal() || neverSawTarget;
    }
}
