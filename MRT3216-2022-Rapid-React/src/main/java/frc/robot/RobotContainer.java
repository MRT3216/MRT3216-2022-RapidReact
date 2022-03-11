package frc.robot;

import java.util.function.BooleanSupplier;

// region Imports

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI.Gamepad;
import frc.robot.OI.OIUtils;
import frc.robot.commands.TeleDrive;
import frc.robot.commands.shooter.RunHopper;
import frc.robot.commands.shooter.RunIntake;
import frc.robot.settings.Constants.Drivetrain;
import frc.robot.settings.RobotMap;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.shooter.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import io.github.oblarg.oblog.Logger;

// endregion

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // region Fields

    private static RobotContainer instance;
    private SwerveSubsystem driveSystem;
    private IntakeSubsystem intakeSystem;
    private ShooterSubsystem shooterSystem;
    private HopperSubsystem hopperSystem;
    private Gamepad controller;
    private LimelightSubsystem limelightSystem;
    private ColorSensorSubsystem colorSensorSystem;

    // endregion

    // region Oblog Logging and Config

    // endregion

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {

        this.initSubsystems();

        // The first argument is the root container
        // The second argument is whether logging and config should be given separate
        // tabs
        Logger.configureLoggingAndConfig(this, false);

        // Configure the button bindings
        configureButtonBindings();
    }

    public void initSubsystems() {
        this.driveSystem = new SwerveSubsystem();
        this.shooterSystem = new ShooterSubsystem();
        this.hopperSystem = new HopperSubsystem();
        this.controller = new Gamepad(RobotMap.DRIVE_STATION.USB_XBOX_CONTROLLER);
        this.limelightSystem = LimelightSubsystem.getInstance();
        this.colorSensorSystem = new ColorSensorSubsystem();
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
                    () -> OIUtils.modifyAxis(-controller.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                    () -> OIUtils.modifyAxis(-controller.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                    () -> OIUtils.modifyAxis(-controller.getRightX())
                            * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    true));

        }

        if (intakeSystem != null && controller != null) {
            BooleanSupplier bs = () -> true;
            controller.A.whileHeld(new RunIntake(this.intakeSystem, bs));
        }
        if (hopperSystem != null && controller != null) {
            BooleanSupplier bs = () -> true;
            controller.B.whileHeld(new RunHopper(this.hopperSystem, bs));
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
