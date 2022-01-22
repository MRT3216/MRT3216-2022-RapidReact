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
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.RobotMap;

public class SwerveSubsystem extends SubsystemBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * SdsModuleConfigurations.MK3_STANDARD.getDriveReduction()
            * SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI * .25;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d());

    // connected over I2C
    private final AHRS m_navx = new AHRS(RobotMap.ROBOT.SENSORS.navx, (byte) 200); // NavX

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
    private final SwerveModule[] m_swerveModules;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public SwerveSubsystem() {
        System.out.println("Max velocity: " + MAX_VELOCITY_METERS_PER_SECOND);
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                // This parameter is optional, but will allow you to see the current state of
                // the module on the dashboard.
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0,
                        0),
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
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2,
                        0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD, RIGHT_FRONT_DRIVE, RIGHT_FRONT_ANGLE,
                RIGHT_FRONT_CANCODER, RIGHT_FRONT_STEER_OFFSET);

        m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4,
                        0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD, LEFT_REAR_DRIVE, LEFT_REAR_ANGLE,
                LEFT_REAR_CANCODER, LEFT_REAR_STEER_OFFSET);

        m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6,
                        0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD, RIGHT_REAR_DRIVE, RIGHT_REAR_ANGLE,
                RIGHT_REAR_CANCODER, RIGHT_REAR_STEER_OFFSET);

        m_swerveModules = new SwerveModule[] { m_frontLeftModule, m_frontRightModule,
                m_backLeftModule,
                m_backRightModule };

        // tab.addNumber("Gyroscope Angle", () -> (360 - m_navx.getYaw()));
        // tab.addNumber("Pose X", () -> m_odometry.getPoseMeters().getX());
        // tab.addNumber("Pose Y", () -> m_odometry.getPoseMeters().getY());
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        System.out.println("Zeroing gyroscope...");
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
        return Rotation2d.fromDegrees(360 - m_navx.getYaw());
    }

    public Pose2d getCurrentRobotPose() {
        return m_odometry.getPoseMeters();
    }

    public void setCurrentRobotPose(Pose2d pose) {
        m_odometry.resetPosition(pose, getGyroscopeRotation());
    }

    public void stop() {
        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].set(0, m_swerveModules[i].getSteerAngle());
        }
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].set(
                    states[i].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
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
        return new SwerveModuleState(module.getDriveVelocity(), Rotation2d.fromDegrees(module.getSteerAngle()));
    }
}