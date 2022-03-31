package frc.robot;

// region Imports

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.OI.Gamepad;
import frc.robot.OI.OIUtils;
import frc.robot.commands.TeleDrive;
import frc.robot.commands.shooter.AimAndFireCargo;
import frc.robot.commands.shooter.FireCargo;

import frc.robot.commands.shooter.IndexCargo;
import frc.robot.commands.shooter.RunHopper;
import frc.robot.commands.shooter.RunIndexer;
import frc.robot.commands.shooter.RunIntake;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Auto;
import frc.robot.settings.Constants.Drivetrain;
import frc.robot.settings.Constants.LimeLight.CameraStream;
import frc.robot.settings.Constants.LimeLight.LEDMode;
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
    // @Config(name = "Flywheel P", tabName = "Tuning", defaultValueNumeric =
    // Constants.Shooter.Flywheel.kP, methodName = "setPValue", methodTypes = {
    // double.class }, rowIndex = 0, columnIndex = 8)
    // @Config(name = "Flywheel I", tabName = "Tuning", defaultValueNumeric =
    // Constants.Shooter.Flywheel.kI, methodName = "setIValue", methodTypes = {
    // double.class }, rowIndex = 1, columnIndex = 8)
    // @Config(name = "Flywheel D", tabName = "Tuning", defaultValueNumeric =
    // Constants.Shooter.Flywheel.kD, methodName = "setDValue", methodTypes = {
    // double.class }, rowIndex = 1, columnIndex = 9)
    // @Config(name = "Flywheel F", tabName = "Tuning", defaultValueNumeric =
    // Constants.Shooter.Flywheel.kF, methodName = "setFValue", methodTypes = {
    // double.class }, rowIndex = 0, columnIndex = 9)
    @Log(name = "Flywheel Velocity", tabName = "Tuning", methodName = "getRPM", rowIndex = 2, columnIndex = 8)
    @Log.Graph(name = "Flywheel Velocity G", tabName = "Tuning", methodName = "getRPM", width = 4, height = 3, rowIndex = 0, columnIndex = 4)
    // @Log.Graph(name = "Flywheel Filter Value", tabName = "Tuning", methodName =
    // "getFilterValue", width = 4, height = 3, rowIndex = 3, columnIndex = 4)
    private ShooterSubsystem shooterSystem;
    // @Config(name = "Indexer P", tabName = "Tuning", defaultValueNumeric =
    // Constants.Shooter.Indexer.kP, methodName = "setPValue", methodTypes = {
    // double.class }, rowIndex = 3, columnIndex = 8)
    // @Config(name = "Indexer I", tabName = "Tuning", defaultValueNumeric =
    // Constants.Shooter.Indexer.kI, methodName = "setIValue", methodTypes = {
    // double.class }, rowIndex = 4, columnIndex = 8)
    // @Config(name = "Indexer D", tabName = "Tuning", defaultValueNumeric =
    // Constants.Shooter.Indexer.kD, methodName = "setDValue", methodTypes = {
    // double.class }, rowIndex = 4, columnIndex = 9)
    // @Config(name = "Indexer F", tabName = "Tuning", defaultValueNumeric =
    // Constants.Shooter.Indexer.kF, methodName = "setFValue", methodTypes = {
    // double.class }, rowIndex = 3, columnIndex = 9)
    // @Log(name = "Indexer Velocity", tabName = "Tuning", methodName = "getRPM",
    // rowIndex = 2, columnIndex = 9)
    private IndexerSubsystem indexerSystem;
    private HopperSubsystem hopperSystem;
    private ClimberSubsystem climberSystem;
    private Gamepad controller;
    private ControlStick controlStick;
    @Config(name = "LED Mode", tabName = "Tuning", methodName = "setLEDModeByInt", methodTypes = {
            int.class }, rowIndex = 3, columnIndex = 3)
    @Config(name = "Stream Mode", tabName = "Tuning", methodName = "setStreamByInt", methodTypes = {
            int.class }, rowIndex = 3, columnIndex = 2)
    @Log.BooleanBox(name = "Target Found", methodName = "hasTarget", rowIndex = 0, columnIndex = 3, width = 1, height = 1)
    // @Log.BooleanBox(name = "Target Found", methodName = "hasTarget", rowIndex =
    // 1, columnIndex = 3, width = 1, height = 1)
    @Config.NumberSlider(name = "Dist. Adj.", tabName = "Tuning", defaultValue = Constants.Shooter.kDistanceAdjustmentInMeters, methodName = "setDistanceAdjustmentInMeters", methodTypes = {
            double.class }, min = -2, max = 2, blockIncrement = 0.1, rowIndex = 0, columnIndex = 0)
    @Log.Dial(name = "Hor. Goal Distance", methodName = "getHorizontalGoalDistance", min = -90, max = 90, rowIndex = 0, columnIndex = 6, height = 1, width = 1)
    @Log.Dial(name = "Horizontal Offset", methodName = "getHorizontalOffset", min = -90, max = 90, rowIndex = 0, columnIndex = 4, height = 1, width = 1)
    @Log.Dial(name = "Vertical Offset", methodName = "getVerticalOffset", min = -90, max = 90, rowIndex = 0, columnIndex = 5, height = 1, width = 1)
    private LimelightSubsystem limelightSystem;
    @Log.BooleanBox(name = "Red Detected", methodName = "isRed", rowIndex = 0, columnIndex = 0)
    @Log.BooleanBox(name = "Blue Detected", methodName = "isBlue", rowIndex = 0, columnIndex = 1)
    @Log.BooleanBox(name = "Alliance Ball", methodName = "isAllianceBall", rowIndex = 1, columnIndex = 0)
    @Log.BooleanBox(name = "Opponent Ball", methodName = "isOpponentBall", rowIndex = 1, columnIndex = 1)
    private ColorSensorSubsystem colorSensorSystem;
    // @Log(name = "Hood Position", tabName = "Tuning", methodName =
    // "getMeasurement", rowIndex = 3, columnIndex = 2)
    // @Config.NumberSlider(name = "Move Hood", tabName = "Tuning", methodName =
    // "setAngle", methodTypes = {
    // double.class }, defaultValue = Constants.Shooter.Hood.hoodReverseLimit, min =
    // Constants.Shooter.Hood.hoodForwardLimit, max =
    // Constants.Shooter.Hood.hoodReverseLimit, rowIndex = 3, columnIndex = 0)
    private HoodSubsystem hoodSystem;
    @Log(name = "1st Ball", methodName = "getBall1String", rowIndex = 2, columnIndex = 0)
    @Log(name = "2nd Ball", methodName = "getBall2String", rowIndex = 2, columnIndex = 1)
    @Log.BooleanBox(name = "Ball in Chute", methodName = "getBallInChute", rowIndex = 1, columnIndex = 2)
    private ShooterStateMachine shooterStateMachine;
    private AutoChooser autoChooser;
    private double autoStartDelayTime;
    private double translationExpo;
    private double rotationExpo;

    // endregion

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        this.autoStartDelayTime = Constants.Auto.kStartDelayTime;
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
        this.controlStick = new ControlStick(RobotMap.DRIVE_STATION.USB_JOYSTICK);
        this.limelightSystem = LimelightSubsystem.getInstance();
        this.limelightSystem.setStream(CameraStream.PiPSecondary);
        this.limelightSystem.setPipeline(0);
        this.colorSensorSystem = ColorSensorSubsystem.getInstance();
        this.shooterStateMachine = ShooterStateMachine.getInstance();
        this.autoChooser = AutoChooser.getInstance();
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

        // Fires cargo (assumes all the aiming has already been completed)
        controller.LB.whileHeld(
                new ParallelCommandGroup(
                        new StartEndCommand(
                                () -> limelightSystem.setLEDMode(LEDMode.ON),
                                () -> limelightSystem.setLEDMode(LEDMode.OFF)),
                        new AimAndFireCargo(this.shooterSystem, this.indexerSystem, this.hopperSystem,
                                this.colorSensorSystem,
                                this.limelightSystem,
                                this.driveSystem, this.hoodSystem)));

        // Runs the intake, hopper, and indexer
        // The indexer and shooter will vary behaviour depending on whether an alliance
        // or opponent ball has been pulled in
        controller.RB.whileHeld(new ParallelCommandGroup(
                new RunIntake(this.intakeSystem, () -> true),
                new RunHopper(this.hopperSystem, () -> true),
                new IndexCargo(this.indexerSystem, () -> this.colorSensorSystem.isAllianceBall()),
                new SpinShooter(this.shooterSystem, () -> true, () -> true)));

        // Turns the Limelight LEDs on, adjusts the hood, and then aims the drivebase if
        // a target is acquired
        controller.A.whenHeld(
                new FireCargo(this.shooterSystem, this.indexerSystem, this.hopperSystem, this.colorSensorSystem,
                        this.limelightSystem));

        // This is to run things in reverse to remove balls
        controller.X.whileHeld(new ParallelCommandGroup(
                new RunHopper(this.hopperSystem, () -> false),
                new RunIndexer(this.indexerSystem, () -> false, () -> true, () -> false),
                new SpinShooter(this.shooterSystem, () -> false, () -> false)));

        // Resets the Robots Odometry and Gyro values
        controller.Y.whenPressed(() -> RobotContainer.getInstance().getDriveSystem().resetGyroAndOdometry(true),
                driveSystem);

        controller.RightJoy.whenPressed(() -> climberSystem.invert());
        controller.LeftJoy.whenPressed(() -> climberSystem.tethered(!climberSystem.isTethered()));

        climberSystem.setDefaultCommand(new FunctionalCommand(
                () -> {
                }, // OnInit: do nothing
                () -> {
                    if (climberSystem.isTethered()) {
                        climberSystem.runMotors(controller.getRightTriggerAxis()
                                - controller.getLeftTriggerAxis());
                    } else {
                        climberSystem.runLeftMotor(controller.getLeftTriggerAxis());
                        climberSystem.runRightMotor(controller.getRightTriggerAxis());
                    }
                }, // OnExecute:
                   // call
                   // run
                   // motors
                interrupted -> climberSystem.stop(), // OnEnd: stop motors
                () -> false, // IsFinished: never finish
                climberSystem)); // Required subsystem

        controlStick.Trigger.whenHeld(
                new ParallelCommandGroup(
                        new RunIntake(this.intakeSystem, () -> true),
                        new RunHopper(this.hopperSystem, () -> true),
                        new IndexCargo(this.indexerSystem, () -> this.colorSensorSystem.isAllianceBall()),
                        new SpinShooter(this.shooterSystem, () -> true, () -> true)));

        controlStick.button2.whenHeld(
                new ParallelCommandGroup(
                        new RunHopper(this.hopperSystem, () -> false),
                        new RunIndexer(this.indexerSystem, () -> false, () -> true, () -> false),
                        new SpinShooter(this.shooterSystem, () -> false, () -> false)));

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
        return this.autoChooser.getAutoCommand();
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

    // @Config.NumberSlider(name = "F Intake Speed", tabName = "Tuning",
    // defaultValue = Constants.Intake.kForwardIntakeSpeed, min = 0, max = 1,
    // blockIncrement = 0.05, rowIndex = 0, columnIndex = 0, height = 1, width = 1)
    // public void setForwardPercentOutput(double output) {
    // this.intakeSystem.setForwardPercentOutput(output);
    // }

    // @Config.NumberSlider(name = "R Intake Speed", tabName = "Tuning",
    // defaultValue = Constants.Intake.kReverseIntakeSpeed, min = 0, max = 1,
    // blockIncrement = 0.05, rowIndex = 1, columnIndex = 0, height = 1, width = 1)
    // public void setReversePercentOutput(double output) {
    // this.intakeSystem.setReversePercentOutput(output);
    // }

    // @Config.NumberSlider(name = "Hopper Speed", tabName = "Tuning", defaultValue
    // = Constants.Shooter.Hopper.kHopperSpeed, min = 0, max = 1, blockIncrement =
    // 0.05, rowIndex = 0, columnIndex = 1, height = 1, width = 1)
    // public void setPercentOutput(double output) {
    // this.hopperSystem.setPercentOutput(output);
    // }

    // control stick
    /*
     * Fender shot: 3
     * Hangar Shot: 4
     */

    // @Config.NumberSlider(name = "Ind. Shoot RPM", tabName = "Tuning",
    // defaultValue = Constants.Shooter.Indexer.shootingRPM, min = 1000, max = 4000,
    // blockIncrement = 50, rowIndex = 0, columnIndex = 2, height = 1, width = 1)
    // public void setIndexerShootingRPM(double rPM) {
    // this.indexerSystem.setShootingRPM(rPM);
    // }

    // @Config.NumberSlider(name = "Ind. Index RPM", tabName = "Tuning",
    // defaultValue = Constants.Shooter.Indexer.indexingRPM, min = 1000, max = 4000,
    // blockIncrement = 50, rowIndex = 1, columnIndex = 2, height = 1, width = 1)
    // public void setIndexerIndexRPM(double rPM) {
    // this.indexerSystem.setIndexingRPM(rPM);
    // }

    // @Config.NumberSlider(name = "Sho. Shoot RPM", tabName = "Tuning",
    // defaultValue = Constants.Shooter.Flywheel.targetShootingRPM, min = 1000, max
    // = 4000, blockIncrement = 50, rowIndex = 0, columnIndex = 3, height = 1, width
    // = 1)
    // public void setShooterShootingRPM(double rPM) {
    // this.shooterSystem.setShootingRPM(rPM);
    // }

    // @Config.NumberSlider(name = "Sho. Eject RPM", tabName = "Tuning",
    // defaultValue = Constants.Shooter.Flywheel.targetEjectRPM, min = 1000, max =
    // 4000, blockIncrement = 50, rowIndex = 1, columnIndex = 3, height = 1, width =
    // 1)
    // public void setEjectRPM(double rPM) {
    // this.shooterSystem.setEjectRPM(rPM);
    // }

    @Config(name = "Auto Delay", tabName = "Tuning", methodName = "setStartDelayTime", defaultValueNumeric = Auto.kStartDelayTime, methodTypes = {
            double.class }, rowIndex = 4, columnIndex = 3)
    public void setStartDelayTime(double startDelayTime) {
        this.autoStartDelayTime = startDelayTime;
    }

    public double getAutoStartDelayTime() {
        return this.autoStartDelayTime;
    }

    // @Config.NumberSlider(name = "Trans. Expo", tabName = "Tuning", defaultValue =
    // Constants.OI.kTranslationExpo, min = 0, max = 100, blockIncrement = 1,
    // rowIndex = 2, columnIndex = 0, height = 1, width = 1)
    // public void setTranslationExpo(double expo) {
    // this.translationExpo = expo;
    // }

    // @Config.NumberSlider(name = "Rotation Expo", tabName = "Tuning", defaultValue
    // = Constants.OI.kRotationnExpo, min = 0, max = 100, blockIncrement = 1,
    // rowIndex = 2, columnIndex = 1, height = 1, width = 1)
    // public void setRotationExpo(double expo) {
    // this.rotationExpo = expo;
    // }
}