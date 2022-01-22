// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.subsystems.SwerveSubsystem;

/* Given a heading, the robot will travel the trajectory while maintaining that heading */
public class DriveConstantHeadingTrajectory extends DriveTrajectory {

  private final Rotation2d constantHeading;

  /** Creates a new DriveHolonomicTrajectory. */
  public DriveConstantHeadingTrajectory(final SwerveSubsystem swerveSubsystem, Trajectory trajectory, Rotation2d constantHeading) {
    super(swerveSubsystem, trajectory);
    this.constantHeading = constantHeading;
  }

  @Override
  public Rotation2d getHeading(State goalState) {
      return constantHeading;
  }
}
