// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.subsystems.SwerveSubsystem;

/* Given a Trajectory, the robot will drive so that it is always pointing in the direction of motion */
public class DriveDifferentialTrajectory extends DriveTrajectory {


  /** Creates a new DriveHolonomicTrajectory. */
  public DriveDifferentialTrajectory(final SwerveSubsystem swerveSubsystem, Trajectory trajectory) {
    super(swerveSubsystem, trajectory);
  }

  @Override
  public Rotation2d getHeading(State goalState) {
      return goalState.poseMeters.getRotation();
  }
}
