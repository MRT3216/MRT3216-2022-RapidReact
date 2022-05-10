// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.AutoAimAndShoot;
import frc.robot.commands.auto.autoProcedures.FiveBall;
import frc.robot.commands.auto.autoProcedures.HPThreeBall;
import frc.robot.commands.auto.autoProcedures.HPTwoBall;
import frc.robot.commands.auto.autoProcedures.HangerTwoBall;
import frc.robot.settings.Constants.Directories;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.HopperSubsystem;
import frc.robot.subsystems.shooter.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.function.Supplier;

/** Add your docs here. */
public class AutoChooser {

    private static AutoChooser instance;
    private Dictionary<String, Trajectory> trajectories;
    private SendableChooser<Supplier<Command>> chooser;
    private SwerveSubsystem swerveSystem;
    private LimelightSubsystem limelightSystem;
    private ShooterSubsystem shooterSystem;
    private IndexerSubsystem indexerSystem;
    private HopperSubsystem hopperSystem;
    private ColorSensorSubsystem colorSensorSystem;
    private HoodSubsystem hoodSystem;
    private IntakeSubsystem intakeSystem;

    private AutoChooser() {
        chooser = new SendableChooser<>();
        chooser.setDefaultOption("Do Nothing", () -> new WaitCommand(0));
        this.swerveSystem = SwerveSubsystem.getInstance();
        this.limelightSystem = LimelightSubsystem.getInstance();
        this.shooterSystem = ShooterSubsystem.getInstance();
        this.indexerSystem = IndexerSubsystem.getInstance();
        this.hopperSystem = HopperSubsystem.getInstance();
        this.colorSensorSystem = ColorSensorSubsystem.getInstance();
        this.hoodSystem = HoodSubsystem.getInstance();
        this.intakeSystem = IntakeSubsystem.getInstance();
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
                }
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + files[i].toString(), ex.getStackTrace());
            }
        }
    }

    public void populateAutoChooser() {
        chooser.addOption("Just shoot",
                () -> new ConditionalCommand(
                        new AutoAimAndShoot(shooterSystem, indexerSystem, hopperSystem, colorSensorSystem,
                                limelightSystem, swerveSystem, hoodSystem, 1),
                        new AutoAimAndShoot(shooterSystem, indexerSystem, hopperSystem, colorSensorSystem,
                                limelightSystem, swerveSystem, hoodSystem, 1),
                        () -> swerveSystem.gyroConnected()
                ));
        chooser.addOption("Human two ball",
                () -> new ConditionalCommand(
                        new HPTwoBall(this.swerveSystem, this.indexerSystem, this.colorSensorSystem, this.hopperSystem,
                                this.intakeSystem, this.shooterSystem, this.limelightSystem, this.hoodSystem,
                                RobotContainer.getInstance().getAutoStartDelayTime()),
                        new AutoAimAndShoot(shooterSystem, indexerSystem, hopperSystem, colorSensorSystem,
                                limelightSystem, swerveSystem, hoodSystem, 1),
                        () -> swerveSystem.gyroConnected()
                ));
        chooser.addOption("Hanger two ball",
                () -> new ConditionalCommand(
                        new HangerTwoBall(this.swerveSystem, this.indexerSystem, this.colorSensorSystem, this.hopperSystem,
                                this.intakeSystem, this.shooterSystem, this.limelightSystem, this.hoodSystem,
                                RobotContainer.getInstance().getAutoStartDelayTime()),
                        new AutoAimAndShoot(shooterSystem, indexerSystem, hopperSystem, colorSensorSystem,
                                limelightSystem, swerveSystem, hoodSystem, 1),
                        () -> swerveSystem.gyroConnected()
                ));
        chooser.addOption("Human three ball",
                () -> new ConditionalCommand(
                        new HPThreeBall(this.swerveSystem, this.indexerSystem, this.colorSensorSystem, this.hopperSystem,
                                this.intakeSystem, this.shooterSystem, this.limelightSystem, this.hoodSystem,
                                RobotContainer.getInstance().getAutoStartDelayTime()),
                        new AutoAimAndShoot(shooterSystem, indexerSystem, hopperSystem, colorSensorSystem,
                                limelightSystem, swerveSystem, hoodSystem, 1),
                        () -> swerveSystem.gyroConnected()
                ));
        chooser.addOption("Five ball",
        () -> new ConditionalCommand(
                new FiveBall(this.swerveSystem, this.indexerSystem, this.colorSensorSystem, this.hopperSystem,
                        this.intakeSystem, this.shooterSystem, this.limelightSystem, this.hoodSystem,
                        RobotContainer.getInstance().getAutoStartDelayTime()),
                new AutoAimAndShoot(shooterSystem, indexerSystem, hopperSystem, colorSensorSystem,
                        limelightSystem, swerveSystem, hoodSystem, 1),
                () -> swerveSystem.gyroConnected()
        ));

        SmartDashboard.putData(chooser);
    }

    public Command getAutoCommand() {
        return chooser.getSelected().get();
    }
}