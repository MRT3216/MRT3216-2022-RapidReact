package frc.robot;

// region Imports

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI.Gamepad;
import frc.robot.OI.OIUtils;
import frc.robot.commands.TeleDrive;
import frc.robot.settings.Constants;
import frc.robot.settings.RobotMap;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

// endregion

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Loggable {
    // region Fields

    private static RobotContainer instance;
    private SwerveSubsystem driveSystem;

    private CompressorSubsystem compressorSystem;
    private Gamepad controller;
    private LimelightSubsystem limelightSystem;

    // region Oblog Logging and Config

    @Log.PowerDistribution(name = "PDB", rowIndex = 2, columnIndex = 4, height = 4)
    private final PowerDistribution pdb;

    // endregion
    // endregion

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        this.controller = new Gamepad(RobotMap.DRIVE_STATION.USB_XBOX_CONTROLLER);
        this.driveSystem = new SwerveSubsystem();

        this.pdb = new PowerDistribution();
        this.limelightSystem = LimelightSubsystem.getInstance();

        this.initSubsystems();
        Logger.configureLoggingAndConfig(this, false);

        // Configure the button bindings
        configureButtonBindings();
    }

    public void initSubsystems() {
        if (compressorSystem != null) {
            compressorSystem.start();
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        if (driveSystem != null && controller != null) {
            driveSystem.setDefaultCommand(new TeleDrive(
                    driveSystem,
                    () -> OIUtils.modifyAxis(-controller.getLeftY()) * SwerveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                    () -> OIUtils.modifyAxis(-controller.getLeftX()) * SwerveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                    () -> OIUtils.modifyAxis(controller.getRightX())
                            * SwerveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    true));
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoChooser.getInstance().getAutoCommand();
    }

    @Config(name = "Auto Drive Time", defaultValueNumeric = 1, rowIndex = 0, columnIndex = 4)
    private void setAutoDriveTime(final double driveTime) {
        Constants.Auto.driveTime = driveTime;
    }

    @Config(name = "Auto Delay", defaultValueNumeric = 0, rowIndex = 0, columnIndex = 5)
    private void setAutoDelayTime(final double driveTime) {
        Constants.Auto.delayTime = driveTime;
    }

    @Config(name = "F Intake Speed", defaultValueNumeric = 0.55, rowIndex = 0, columnIndex = 3)
    private void setForwardIntakeSpeed(final double speed) {
        Constants.Intake.kForwardIntakeSpeed = speed;
    }

    @Config(name = "R Intake Speed", defaultValueNumeric = 0.55, rowIndex = 1, columnIndex = 3)
    private void setReverseIntakeSpeed(final double speed) {
        Constants.Intake.kReverseIntakeSpeed = speed;
    }

    public static RobotContainer getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new RobotContainer();
        }
        return instance;
    }

    public SwerveSubsystem getDriveSystem() {
        return driveSystem;
    }
}
