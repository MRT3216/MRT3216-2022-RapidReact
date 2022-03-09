// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Dictionary;
import java.util.Hashtable;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.DriveConstantHeadingTrajectory;
import frc.robot.commands.auto.DriveHolonomicTrajectory;
import frc.robot.commands.auto.DriveSupplyHeadingTrajectory;
import frc.robot.settings.Constants.Directories;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoChooser {

    private static AutoChooser instance;
    private Dictionary<String, Trajectory> trajectories;
    private SendableChooser<Command> chooser;
    private RobotContainer robotContainer;
    private SwerveSubsystem swerveSystem;
    private LimelightSubsystem limelightSystem;

    private AutoChooser() {
        chooser = new SendableChooser<>();
        chooser.setDefaultOption("Do Nothing", new WaitCommand(0));
        this.robotContainer = RobotContainer.getInstance();
        this.swerveSystem = robotContainer.getDriveSystem();
        this.limelightSystem = LimelightSubsystem.getInstance();
    }

    public static AutoChooser getInstance() {
        if (instance == null) {
            instance = new AutoChooser();
        }
        return instance;
    }

    public void readTrajectories() {
        File pathsDirectory = new File(Directories.pathsDirectory);
        File[] files = pathsDirectory.listFiles();
        this.trajectories = new Hashtable<String, Trajectory>();
        for (int i = 0; i < files.length; i++) {
            try {
                Path trajectoryPath = files[i].toPath();
                System.out.println("PATH " + i + ": " + trajectoryPath.toString());
                if (files[i].isFile()) {
                    trajectories.put(files[i].getName(), TrajectoryUtil.fromPathweaverJson(trajectoryPath));
                } else {
                    continue;
                }
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + files[i].toString(), ex.getStackTrace());
            }
        }
    }

    public void populateAutoChooser() {/*
                                        * chooser.addOption("Crazy, Holonomic Trajectory",
                                        * new DriveHolonomicTrajectory(swerveSystem, PathPlanner.loadPath("Crazy", 2,
                                        * 1)));
                                        * chooser.addOption("Straight, Holonomic Trajectory",
                                        * new DriveHolonomicTrajectory(swerveSystem, PathPlanner.loadPath("Straight",
                                        * 1, .5)));
                                        * chooser.addOption("Square Path, Holonomic Trajectory",
                                        * new DriveHolonomicTrajectory(swerveSystem, PathPlanner.loadPath("Square 1",
                                        * 2, 1)));
                                        * chooser.addOption("Square Path, Constant Heading", new
                                        * DriveConstantHeadingTrajectory(swerveSystem,
                                        * PathPlanner.loadPath("Square 1", 2, 1), Rotation2d.fromDegrees(45)));
                                        * chooser.addOption("Bounce Path, Differential Trajectory",
                                        * new DriveDifferentialTrajectory(swerveSystem,
                                        * PathPlanner.loadPath("Bounce 1", 2, 1)));
                                        */
        chooser.addOption("3, Holonomic Trajectory",
                new DriveHolonomicTrajectory(swerveSystem, PathPlanner.loadPath("3", 2, 1))
                        .andThen(new DriveConstantHeadingTrajectory(swerveSystem, PathPlanner.loadPath("2", 2, 1),
                                Rotation2d.fromDegrees(60))));
        chooser.addOption("2, Holonomic Trajectory",
                new DriveConstantHeadingTrajectory(swerveSystem, PathPlanner.loadPath("2", 2, 1),
                        Rotation2d.fromDegrees(60)));
        chooser.addOption("Bounce Path, Holonomic Trajectory",
                new DriveHolonomicTrajectory(swerveSystem, PathPlanner.loadPath("Bounce 1", 2, 1)));
        chooser.addOption("Bounce Path, Constant Heading", new DriveConstantHeadingTrajectory(swerveSystem,
                PathPlanner.loadPath("Bounce 1", 2, 1), Rotation2d.fromDegrees(45)));
        /*
         * chooser.addOption("Crazy, Targetted",
         * new DriveSupplyHeadingTrajectory(swerveSystem, PathPlanner.loadPath("Crazy",
         * 2, 1),
         * () -> (limelightSystem.hasTarget() ? limelightSystem.getHorizontalOffset()
         * : swerveSystem.getGyroscopeRotation().getDegrees())));
         */
        chooser.addOption("Side 2 Side, Holonomic",
                new DriveHolonomicTrajectory(swerveSystem, PathPlanner.loadPath("Side", 2, 1)));
        chooser.addOption("Side 2 Side, Targetted",
                new DriveSupplyHeadingTrajectory(swerveSystem, PathPlanner.loadPath("Side", 2, 1),
                        () -> (limelightSystem.hasTarget() ? limelightSystem.getHorizontalOffset()
                                : swerveSystem.getGyroscopeRotation().getDegrees())));

        SmartDashboard.putData(chooser);
    }

    public Command getAutoCommand() {
        return chooser.getSelected();
    }
}