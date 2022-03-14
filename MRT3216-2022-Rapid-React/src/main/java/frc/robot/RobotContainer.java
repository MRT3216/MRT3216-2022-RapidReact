package frc.robot;

// region Imports

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.OI.Gamepad;
import frc.robot.OI.OIUtils;
import frc.robot.commands.TeleDrive;
import frc.robot.commands.shooter.IndexCargo;
import frc.robot.commands.shooter.RunHopper;
import frc.robot.commands.shooter.RunIndexer;
import frc.robot.commands.shooter.RunIntake;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Drivetrain;
import frc.robot.settings.RobotMap;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.HopperSubsystem;
import frc.robot.subsystems.shooter.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
// endregion
import io.github.oblarg.oblog.annotations.Log;

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
    private IndexerSubsystem indexerSystem;
    private HopperSubsystem hopperSystem;
    private Gamepad controller;
    private LimelightSubsystem limelightSystem;
    private ColorSensorSubsystem colorSensorSystem;
    private HoodSubsystem hoodSystem;
    private double translationExpo;
    private double rotationExpo;    
    private PowerDistribution pdh;

    // endregion

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        this.translationExpo = Constants.OI.kTranslationExpo;

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
        this.intakeSystem = new IntakeSubsystem();
        this.hopperSystem = new HopperSubsystem();
        this.indexerSystem = new IndexerSubsystem();
        // this.hoodSystem = new HoodSubsystem();
        this.shooterSystem = new ShooterSubsystem();
        this.controller = new Gamepad(RobotMap.DRIVE_STATION.USB_XBOX_CONTROLLER);
        this.limelightSystem = LimelightSubsystem.getInstance();
        this.colorSensorSystem = new ColorSensorSubsystem();
        this.pdh = new PowerDistribution();
        pdh.clearStickyFaults();
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
                    () -> OIUtils.modifyAxis(-controller.getLeftY(), this.translationExpo)
                            * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                    () -> OIUtils.modifyAxis(-controller.getLeftX(), this.translationExpo)
                            * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                    () -> OIUtils.modifyAxis(-controller.getRightX(), this.rotationExpo)
                            * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    true));
        }

        controller.RB.whileHeld(new ParallelCommandGroup(new RunIntake(this.intakeSystem, () -> true),
                new RunHopper(this.hopperSystem, () -> true),
                new IndexCargo(this.indexerSystem, () -> this.colorSensorSystem.isAllianceBall()),
                new SpinShooter(this.shooterSystem, () -> true, () -> true)));

        controller.LB.whileHeld(new ParallelCommandGroup(
                new RunHopper(this.hopperSystem, () -> true),
                new RunIndexer(this.indexerSystem, () -> true),
                new SpinShooter(this.shooterSystem, () -> true, () -> false)));

        controller.X.whileHeld(new ParallelCommandGroup(new RunHopper(this.hopperSystem, () -> false),
                new RunIndexer(this.indexerSystem, () -> false),
                new SpinShooter(this.shooterSystem, () -> false, () -> false)));

        controller.A.whenPressed(new Runnable() {
            @Override
            public void run() {
                RobotContainer.getInstance().getDriveSystem().resetGyroAndOdometry(true);
            }

        }, driveSystem);
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

    @Config.NumberSlider(name = "F Intake Speed", defaultValue = Constants.Intake.kForwardIntakeSpeed, min = 0, max = 1, blockIncrement = 0.05, tabName = "RobotContainer", rowIndex = 0, columnIndex = 0, height = 1, width = 1)
    public void setForwardPercentOutput(double output) {
        this.intakeSystem.setForwardPercentOutput(output);
    }

    @Config.NumberSlider(name = "R Intake Speed", defaultValue = Constants.Intake.kReverseIntakeSpeed, min = 0, max = 1, blockIncrement = 0.05, tabName = "RobotContainer", rowIndex = 1, columnIndex = 0, height = 1, width = 1)
    public void setReversePercentOutput(double output) {
        this.intakeSystem.setReversePercentOutput(output);
    }

    @Config.NumberSlider(name = "Hopper Speed", defaultValue = Constants.Shooter.Hopper.kHopperSpeed, min = 0, max = 1, blockIncrement = 0.05, tabName = "RobotContainer", rowIndex = 0, columnIndex = 1, height = 1, width = 1)
    public void setPercentOutput(double output) {
        this.hopperSystem.setPercentOutput(output);
    }

    @Config.NumberSlider(name = "Ind. Shoot RPM", defaultValue = Constants.Shooter.Indexer.shootingRPM, min = 1000, max = 4000, blockIncrement = 50, rowIndex = 0, columnIndex = 2, height = 1, width = 1)
    public void setIndexerShootingRPM(double rPM) {
        this.indexerSystem.setShootingRPM(rPM);
    }

    @Config.NumberSlider(name = "Ind. Index RPM", defaultValue = Constants.Shooter.Indexer.indexingRPM, min = 1000, max = 4000, blockIncrement = 50, rowIndex = 1, columnIndex = 2, height = 1, width = 1)
    public void setIndexerIndexRPM(double rPM) {
        this.indexerSystem.setIndexingRPM(rPM);
    }

    @Config.NumberSlider(name = "Sho. Shoot RPM", defaultValue = Constants.Shooter.Flywheel.shootingRPM, min = 1000, max = 4000, blockIncrement = 50, rowIndex = 0, columnIndex = 3, height = 1, width = 1)
    public void setShooterShootingRPM(double rPM) {
        this.shooterSystem.setShootingRPM(rPM);
    }

    @Config.NumberSlider(name = "Sho. Eject RPM", defaultValue = Constants.Shooter.Flywheel.ejectRPM, min = 1000, max = 4000, blockIncrement = 50, rowIndex = 1, columnIndex = 3, height = 1, width = 1)
    public void setEjectRPM(double rPM) {
        this.shooterSystem.setEjectRPM(rPM);
    }

    @Config.NumberSlider(name = "Trans. Expo", defaultValue = Constants.OI.kTranslationExpo, min = 0, max = 100, blockIncrement = 1, rowIndex = 2, columnIndex = 0, height = 1, width = 1)
    public void setTranslationExpo(double expo) {
        this.translationExpo = expo;
    }

    @Config.NumberSlider(name = "Rotation Expo", defaultValue = Constants.OI.kRotationnExpo, min = 0, max = 100, blockIncrement = 1, rowIndex = 2, columnIndex = 1, height = 1, width = 1)
    public void setRotationExpo(double expo) {
        this.rotationExpo = expo;
    }

    /*
    @Log.VoltageView(name = "Rotation Expo", min = 0, max = 100, rowIndex = 3, columnIndex = 1, height = 1, width = 1)
    public double getVoltage() {
        return pdh.getVoltage();
    }
    */
}
