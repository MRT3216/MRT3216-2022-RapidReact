package frc.robot;

// region Imports

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.OI.Gamepad;
import frc.robot.OI.OIUtils;
import frc.robot.commands.TeleDrive;
import frc.robot.commands.shooter.FireCargo;
import frc.robot.commands.shooter.IndexCargo;
import frc.robot.commands.shooter.RunHopper;
import frc.robot.commands.shooter.RunIndexer;
import frc.robot.commands.shooter.RunIntake;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Drivetrain;
import frc.robot.settings.RobotMap;
import frc.robot.subsystems.ClimberSubsystem;
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
import io.github.oblarg.oblog.annotations.Log;

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
    @Config(name = "Flywheel P", defaultValueNumeric = Constants.Shooter.Flywheel.kP, methodName = "setPValue", methodTypes = {
            double.class }, rowIndex = 0, columnIndex = 8)
    @Config(name = "Flywheel I", defaultValueNumeric = Constants.Shooter.Flywheel.kI, methodName = "setIValue", methodTypes = {
            double.class }, rowIndex = 1, columnIndex = 8)
    @Config(name = "Flywheel D", defaultValueNumeric = Constants.Shooter.Flywheel.kD, methodName = "setDValue", methodTypes = {
            double.class }, rowIndex = 1, columnIndex = 9)
    @Config(name = "Flywheel F", defaultValueNumeric = Constants.Shooter.Flywheel.kF, methodName = "setFValue", methodTypes = {
            double.class }, rowIndex = 0, columnIndex = 9)
    @Log.Graph(name = "Flywheel Velocity G", methodName = "getRPM", width = 4, height = 2, rowIndex = 0, columnIndex = 4)
    @Log(name = "Flywheel Velocity", methodName = "getRPM", rowIndex = 2, columnIndex = 8)
    @Log.Graph(name = "Flywheel Filter Value", methodName = "getFilterValue", width = 4, height = 2, rowIndex = 3, columnIndex = 4)
    private ShooterSubsystem shooterSystem;
    @Config(name = "Indexer P", defaultValueNumeric = Constants.Shooter.Indexer.kP, methodName = "setPValue", methodTypes = {
            double.class }, rowIndex = 3, columnIndex = 8)
    @Config(name = "Indexer I", defaultValueNumeric = Constants.Shooter.Indexer.kI, methodName = "setIValue", methodTypes = {
            double.class }, rowIndex = 4, columnIndex = 8)
    @Config(name = "Indexer D", defaultValueNumeric = Constants.Shooter.Indexer.kD, methodName = "setDValue", methodTypes = {
            double.class }, rowIndex = 4, columnIndex = 9)
    @Config(name = "Indexer F", defaultValueNumeric = Constants.Shooter.Indexer.kF, methodName = "setFValue", methodTypes = {
            double.class }, rowIndex = 3, columnIndex = 9)
    @Log(name = "Indexer Velocity", methodName = "getRPM", rowIndex = 5, columnIndex = 8)
    private IndexerSubsystem indexerSystem;
    private HopperSubsystem hopperSystem;
    private ClimberSubsystem climberSystem;
    private Gamepad controller;
    private LimelightSubsystem limelightSystem;
    @Log.BooleanBox(name = "Red Detected", methodName = "isRed", rowIndex = 3, columnIndex = 0)
    @Log.BooleanBox(name = "Blue Detected", methodName = "isBlue", rowIndex = 3, columnIndex = 1)
    @Log.BooleanBox(name = "Opponent Ball", methodName = "isOpponentBall", rowIndex = 3, columnIndex = 2)
    @Log.BooleanBox(name = "Alliance Ball", methodName = "isAllianceBall", rowIndex = 3, columnIndex = 3)
    private ColorSensorSubsystem colorSensorSystem;
    @Log(name = "Hood Position", methodName = "getMeasurement", rowIndex = 5, columnIndex = 2)
    @Config.NumberSlider(name = "Move Hood", methodName = "setAngle", methodTypes = {
            double.class }, defaultValue = Constants.Shooter.Hood.hoodReverseLimit, min = Constants.Shooter.Hood.hoodForwardLimit, max = Constants.Shooter.Hood.hoodReverseLimit, rowIndex = 5, columnIndex = 0)
    private HoodSubsystem hoodSystem;
    @Log(name = "1st Ball", methodName = "get1stBallString", rowIndex = 4, columnIndex = 0)
    @Log(name = "2nd Ball", methodName = "get2ndBallString", rowIndex = 4, columnIndex = 1)
    private ShooterStateMachine shooterStateMachine;
    private double translationExpo;
    private double rotationExpo;

    // endregion

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        this.translationExpo = Constants.OI.kTranslationExpo;
        this.rotationExpo = Constants.OI.kRotationnExpo;

        this.initSubsystems();

        // The first argument is the root container
        // The second argument is whether logging and config should be given separate
        // tabs
        Logger.configureLoggingAndConfig(this, false);
        // Logger.configureLogging(this);

        // Configure the button bindings
        configureButtonBindings();
    }

    public void initSubsystems() {
        this.driveSystem = SwerveSubsystem.getInstance();
        this.intakeSystem = IntakeSubsystem.getInstance();
        this.hopperSystem = HopperSubsystem.getInstance();
        this.indexerSystem = IndexerSubsystem.getInstance();
        this.hoodSystem = HoodSubsystem.getInstance();
        this.hoodSystem.stop();
        this.climberSystem = ClimberSubsystem.getInstance();
        this.shooterSystem = ShooterSubsystem.getInstance();
        this.controller = new Gamepad(RobotMap.DRIVE_STATION.USB_XBOX_CONTROLLER);
        this.limelightSystem = LimelightSubsystem.getInstance();
        this.colorSensorSystem = ColorSensorSubsystem.getInstance();
        this.shooterStateMachine = ShooterStateMachine.getInstance();
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

        controller.LB.whileHeld(
                new FireCargo(this.shooterSystem, this.indexerSystem, this.hopperSystem, this.colorSensorSystem));

        controller.X.whileHeld(new ParallelCommandGroup(new RunHopper(this.hopperSystem, () -> false),
                new RunIndexer(this.indexerSystem, () -> false, () -> true, () -> false),
                new SpinShooter(this.shooterSystem, () -> false, () -> false)));

        controller.A.whenPressed(new Runnable() {
            @Override
            public void run() {
                RobotContainer.getInstance().getDriveSystem().resetGyroAndOdometry(true);
            }

        }, driveSystem);

        climberSystem.setDefaultCommand(new FunctionalCommand(
                () -> {
                }, // OnInit: do nothing
                () -> climberSystem.runMotors(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()), // OnExedcute:
                                                                                                                   // call
                                                                                                                   // run
                                                                                                                   // motors
                interrupted -> climberSystem.stop(), // OnEnd: stop motors
                () -> false, // IsFinished: never finish
                climberSystem)); // Required subsystem
    }

    public void disablePIDSubsystems() {
        if (hoodSystem != null) {
            hoodSystem.disable();
            hoodSystem.stop();
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

    @Config.NumberSlider(name = "Sho. Shoot RPM", defaultValue = Constants.Shooter.Flywheel.targetShootingRPM, min = 1000, max = 4000, blockIncrement = 50, rowIndex = 0, columnIndex = 3, height = 1, width = 1)
    public void setShooterShootingRPM(double rPM) {
        this.shooterSystem.setShootingRPM(rPM);
    }

    @Config.NumberSlider(name = "Sho. Eject RPM", defaultValue = Constants.Shooter.Flywheel.targetEjectRPM, min = 1000, max = 4000, blockIncrement = 50, rowIndex = 1, columnIndex = 3, height = 1, width = 1)
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
}