// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.settings.Constants.Drivetrain.LEFT_FRONT_STEER_OFFSET;
import static frc.robot.settings.Constants.Drivetrain.LEFT_REAR_STEER_OFFSET;
import static frc.robot.settings.Constants.Drivetrain.RIGHT_FRONT_STEER_OFFSET;
import static frc.robot.settings.Constants.Drivetrain.RIGHT_REAR_STEER_OFFSET;
import static frc.robot.settings.Constants.Drivetrain.TRACKWIDTH_METERS;
import static frc.robot.settings.Constants.Drivetrain.WHEELBASE_METERS;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.LEFT_FRONT_ANGLE;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.LEFT_FRONT_CANCODER;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.LEFT_FRONT_DRIVE;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.LEFT_REAR_ANGLE;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.LEFT_REAR_CANCODER;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.LEFT_REAR_DRIVE;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.RIGHT_FRONT_ANGLE;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.RIGHT_FRONT_CANCODER;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.RIGHT_FRONT_DRIVE;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.RIGHT_REAR_ANGLE;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.RIGHT_REAR_CANCODER;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.RIGHT_REAR_DRIVE;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Auto;
import frc.robot.settings.Constants.Drivetrain;
import frc.robot.settings.Gains;

public class SwerveSubsystem extends SubsystemBase {
    private static SwerveSubsystem instance;
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));

    // private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d());
    // todo: implement SwerveDriveOdometry properly for 2023
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(), null);

    // connected over USB
    private final AHRS m_navx;

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
    private final SwerveModule[] m_swerveModules;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private Gains thetaGains;

    private SwerveSubsystem() {
        m_navx = new AHRS(SerialPort.Port.kUSB1);

        // ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                // This parameter is optional, but will allow you to see the current state of
                // the module on the dashboard.
                // tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2,
                // 4).withPosition(0, 0),
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                // This is the ID of the drive motor
                LEFT_FRONT_DRIVE,
                // This is the ID of the steer motor
                LEFT_FRONT_ANGLE,
                // This is the ID of the steer encoder
                LEFT_FRONT_CANCODER,
                // This is how much the steer encoder is offset from true zero (In our case,
                // zero is facing straight forward)
                LEFT_FRONT_STEER_OFFSET);

        // We will do the same for the other modules
        m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                // tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2,
                // 4).withPosition(2, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD, RIGHT_FRONT_DRIVE, RIGHT_FRONT_ANGLE,
                RIGHT_FRONT_CANCODER, RIGHT_FRONT_STEER_OFFSET);

        m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                // tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2,
                // 4).withPosition(4, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD, LEFT_REAR_DRIVE, LEFT_REAR_ANGLE,
                LEFT_REAR_CANCODER, LEFT_REAR_STEER_OFFSET);

        m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                // tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2,
                // 4).withPosition(6, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD, RIGHT_REAR_DRIVE, RIGHT_REAR_ANGLE,
                RIGHT_REAR_CANCODER, RIGHT_REAR_STEER_OFFSET);

        m_swerveModules = new SwerveModule[] { m_frontLeftModule, m_frontRightModule,
                m_backLeftModule,
                m_backRightModule };

        thetaGains = Auto.kAutoThetaGains;
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        System.out.println("Zeroing Gyroscope");
        m_navx.zeroYaw();
        // Reset the odometry with new 0 heading but same position.
        m_odometry.resetPosition(m_odometry.getPoseMeters(), new Rotation2d());
    }

    /**
     * Calibrates the gyroscope. This should only be called on robotinit because
     * it takes some time to run.
     */
    public void calibrateGyroscope() {
        m_navx.calibrate();
    }

    public Rotation2d getGyroscopeRotation() {
        // // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(-m_navx.getYaw());
    }

    public Pose2d getCurrentRobotPose() {
        return m_odometry.getPoseMeters();
    }

    public void setCurrentRobotPose(Pose2d pose) {
        m_odometry.resetPosition(pose, getGyroscopeRotation());
    }

    public void stop() {
        for (SwerveModule m_swerveModule : m_swerveModules) {
            m_swerveModule.set(0, m_swerveModule.getSteerAngle());
        }
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);

        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].set(
                    states[i].speedMetersPerSecond / Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Drivetrain.MAX_VOLTAGE,
                    states[i].angle.getRadians());
        }

        var gyroAngle = this.getGyroscopeRotation();

        // Update the pose
        m_odometry.update(gyroAngle, getState(m_frontLeftModule), getState(m_frontRightModule),
                getState(m_backLeftModule), getState(m_backRightModule));
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */

    public SwerveModuleState getState(SwerveModule module) {
        return new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle()));
    }

    public boolean gyroConnected() {
        return this.m_navx.isConnected();
    }

    public Gains getThetaGains() {
        return this.thetaGains;
    }

    // @Config.ToggleButton(name = "ResetGyroAndOdometry", defaultValue = false,
    // rowIndex = 3, columnIndex = 0, height = 1, width = 2)
    public void resetGyroAndOdometry(boolean _input) {
        if (_input) {
            this.zeroGyroscope();
            _input = false;
        }
    }

    public boolean navXIsConnected() {
        return m_navx.isConnected();
    }

    // region Logging

    // @Log.Gyro(name = "Robot Angle", rowIndex = 0, columnIndex = 3)
    // private AHRS getGyro() {
    // return m_navx;
    // }

    // @Log.NumberBar(name = "FL Velocity", min = -5, max = 5, rowIndex = 0,
    // columnIndex = 2, height = 1, width = 1)
    // public double getFrontLeftSpeed() {
    // return m_frontLeftModule.getDriveVelocity();
    // }

    // @Log.Dial(name = "FL Angle", min = -90, max = 90, rowIndex = 0, columnIndex =
    // 1, height = 1, width = 1)
    // public double getFrontLeftAngle() {
    // return Math.IEEEremainder(Math.toDegrees(m_frontLeftModule.getSteerAngle()),
    // 180);
    // }

    // @Log.NumberBar(name = "FR Velocity", min = -5, max = 5, rowIndex = 0,
    // columnIndex = 5, height = 1, width = 1)
    // public double getFrontRightSpeed() {
    // return m_frontRightModule.getDriveVelocity();
    // }

    // @Log.Dial(name = "FR Angle", min = -90, max = 90, rowIndex = 0, columnIndex =
    // 6, height = 1, width = 1)
    // public double getFrontRightAngle() {
    // return Math.IEEEremainder(Math.toDegrees(m_frontRightModule.getSteerAngle()),
    // 180);
    // }

    // @Log.Dial(name = "BL Angle", min = -90, max = 90, rowIndex = 1, columnIndex =
    // 1, height = 1, width = 1)
    // public double getBackLeftAngle() {
    // return Math.IEEEremainder(Math.toDegrees(m_backLeftModule.getSteerAngle()),
    // 180);
    // }

    // @Log.NumberBar(name = "BL Velocity", min = -5, max = 5, rowIndex = 1,
    // columnIndex = 2, height = 1, width = 1)
    // public double getBackLeftSpeed() {
    // return m_backLeftModule.getDriveVelocity();
    // }

    // @Log.NumberBar(name = "BR Velocity", min = -5, max = 5, rowIndex = 1,
    // columnIndex = 5, height = 1, width = 1)
    // public double getBackRightSpeed() {
    // return m_backRightModule.getDriveVelocity();
    // }

    // @Log.Dial(name = "BR Angle", min = -90, max = 90, rowIndex = 1, columnIndex =
    // 6, height = 1, width = 1)
    // public double getBackRightAngle() {
    // return Math.IEEEremainder(Math.toDegrees(m_backRightModule.getSteerAngle()),
    // 180);
    // }

    // @Log(name = "x-Position", rowIndex = 2, columnIndex = 6, height = 1, width =
    // 1)
    // public double getYPos() {
    // return m_odometry.getPoseMeters().getY();
    // }

    // @Log(name = "y-Position", rowIndex = 3, columnIndex = 6, height = 1, width =
    // 1)
    // public double getXPos() {
    // return m_odometry.getPoseMeters().getX();
    // }

    // @Log(name = "theta-Position", rowIndex = 4, columnIndex = 6, height = 1,
    // width = 1)
    // public double getThetaPos() {
    // return m_odometry.getPoseMeters().getRotation().getDegrees();
    // }

    // @Log(name = "x-Velocity", rowIndex = 2, columnIndex = 7, height = 1, width =
    // 1)
    // public double getRobotXVelocity() {
    // return m_chassisSpeeds.vxMetersPerSecond;
    // }

    // @Log(name = "y-Velocity", rowIndex = 3, columnIndex = 7, height = 1, width =
    // 1)
    // public double getRobotYVelocity() {
    // return m_chassisSpeeds.vyMetersPerSecond;
    // }

    // @Log(name = "theta-Velocity", rowIndex = 4, columnIndex = 7, height = 1,
    // width = 1)
    // public double getRobotThetaVelocity() {
    // return m_chassisSpeeds.omegaRadiansPerSecond;
    // }

    // @Log.BooleanBox(name = "Gyro Int?", rowIndex = 0, columnIndex = 0)
    // public boolean getGyroInterference() {
    // return this.m_navx.isMagneticDisturbance();
    // }

    // @Config.ToggleButton(name = "ResetPosition", defaultValue = false, rowIndex =
    // 4, columnIndex = 0, height = 1, width = 2)
    // public void resetPosition(boolean _input) {
    // if (_input) {
    // // Reset the odometry with new 0 heading and zero Position.
    // m_odometry.resetPosition(new Pose2d(), new Rotation2d());
    // _input = false;
    // }
    // }

    // @Config.NumberSlider(name = "Theta P", tabName = "Tuning", defaultValue =
    // Auto.kThetaP, min = 0, max = 20, rowIndex = 5, columnIndex = 0, height = 1,
    // width = 1)
    // public void setThetaP(double thetaP) {
    // this.thetaGains.kP = thetaP;
    // }

    // @Config.NumberSlider(name = "Theta I", tabName = "Tuning", defaultValue =
    // Auto.kThetaI, min = 0, max = 1, rowIndex = 5, columnIndex = 1, height = 1,
    // width = 1)
    // public void setThetaI(double thetaI) {
    // this.thetaGains.kI = thetaI;
    // }

    // @Config.NumberSlider(name = "Theta D", tabName = "Tuning", defaultValue =
    // Auto.kThetaD, min = 0, max = 1, rowIndex = 5, columnIndex = 2, height = 1,
    // width = 1)
    // public void setThetaD(double thetaD) {
    // this.thetaGains.kD = thetaD;
    // }

    // @Log.Graph(name = "Gyro Angle", width = 4, height = 2, rowIndex = 2,
    // columnIndex = 2)
    // public double getGyroDegrees() {
    // return this.getGyroscopeRotation().getDegrees();
    // }

    // endregion

    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new SwerveSubsystem();
        }
        return instance;
    }
}