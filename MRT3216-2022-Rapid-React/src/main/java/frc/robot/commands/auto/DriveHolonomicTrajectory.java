// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.subsystems.SwerveSubsystem;

/* Given a Path Planner Trajectory, the robot will drive with separate translation and rotation vectors. */
public class DriveHolonomicTrajectory extends DriveTrajectory {
  /** Creates a new DriveHolonomicTrajectory. */
  public DriveHolonomicTrajectory(final SwerveSubsystem swerveSubsystem, PathPlannerTrajectory trajectory) {
    super(swerveSubsystem, trajectory);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public Rotation2d getHeading(State goalState) {
      var goal = (PathPlannerState) goalState;
      return goal.holonomicRotation;
  }
}
