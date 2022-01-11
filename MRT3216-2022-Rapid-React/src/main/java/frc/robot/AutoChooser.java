// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Dictionary;
import java.util.Hashtable;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.settings.Constants.Directories;

/** Add your docs here. */
public class AutoChooser {

    private static AutoChooser instance;
    private Dictionary<String, Trajectory> trajectories;
    private SendableChooser<Command> chooser;
    private RobotContainer robotContainer;

    private AutoChooser() {
        this.robotContainer = RobotContainer.getInstance();
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

    public void populateAutoChooser() {
        chooser = new SendableChooser<>();
        chooser.setDefaultOption("Do Nothing", new WaitCommand(0));
        // TODO: Fill this in with proper options
        /*
         * chooser.addOption("Auto Drive and Shoot", new
         * WaitCommand(robotContainer.getAutoDelayTime()).andThen( new
         * AutoDriveForTime(driveSystem, robotContainer.getAutoDriveTime()).andThen(new
         * WaitCommand(1), new AutoAimAndShoot(neopixelRingSystem, visionLightsSystem,
         * driveSystem, hoodSystem, () -> ntController.getYaw(), () ->
         * ntController.getPitch(), () -> ntController.isValidTarget(), shooterSystem,
         * indexerSystem, hopperSystem, intakeSystem))));
         */
        SmartDashboard.putData(chooser);
    }
    /*
     * public Command generateAutoPath(String trajName) { final Trajectory
     * trajectory = trajectories.get(trajName);
     * 
     * // RamseteCommand ramseteCommand = new RamseteCommand(trajectory,
     * driveSystem::getPose, // new RamseteController(Auto.kRamseteB,
     * Auto.kRamseteZeta), // new
     * SimpleMotorFeedforward(Drive.ksVolts.getAsDouble(),
     * Drive.kvVoltSecondsPerMeter.getAsDouble(), //
     * Drive.kaVoltSecondsSquaredPerMeter.getAsDouble()), // Drive.kDriveKinematics,
     * driveSystem::getWheelSpeeds, // new
     * PIDController(Drive.kPDriveVel.getAsDouble(), 0, 0), // new
     * PIDController(Drive.kPDriveVel.getAsDouble(), 0, 0), // // RamseteCommand
     * passes volts to the callback // driveSystem::tankDriveVolts, driveSystem);
     * 
     * // // Run path following command // return new ShiftTransmission(driveSystem,
     * !Constants.IS_LOW_GEAR) // .andThen(() ->
     * driveSystem.resetOdometry(trajectory.getInitialPose())).andThen(
     * ramseteCommand); }
     */

    public Command getAutoCommand() {
        return chooser.getSelected();
    }
}