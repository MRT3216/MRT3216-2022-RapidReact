// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.subsystems.SwerveSubsystem;

/* Given a heading, the robot will travel the trajectory while maintaining that heading */
public class DriveSupplyHeadingTrajectory extends DriveTrajectory {

    private final DoubleSupplier headingSupplier;

    /** Creates a new DriveHolonomicTrajectory. */
    public DriveSupplyHeadingTrajectory(final SwerveSubsystem swerveSubsystem, Trajectory trajectory,
            DoubleSupplier headingSupplier) {
        super(swerveSubsystem, trajectory);
        this.headingSupplier = headingSupplier;
    }

    @Override
    public Rotation2d getHeading(State goalState) {
        return Rotation2d.fromDegrees(this.headingSupplier.getAsDouble());
    }
}
