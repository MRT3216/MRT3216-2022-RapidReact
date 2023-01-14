package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.SwerveModule;
import frc.robot.settings.Configurations;
import frc.robot.settings.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Gains;
import frc.robot.settings.RobotMap;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] swerveModules;
    public AHRS gyro;

    static SwerveSubsystem instance;

    private Gains thetaGains;

    private SwerveSubsystem() {
        gyro = new AHRS(RobotMap.ROBOT.SENSORS.NAVX);
        gyro.calibrate();
        zeroGyro();
        // Initializes modules 1 by 1, in the order of front left -> front right -> back left -> back right
        swerveModules = new SwerveModule[] {

                //front left module
                new SwerveModule(0, new Configurations.SwerveModuleConstants(
                        RobotMap.ROBOT.DRIVETRAIN.LEFT_FRONT_DRIVE,
                        RobotMap.ROBOT.DRIVETRAIN.LEFT_FRONT_ANGLE,
                        RobotMap.ROBOT.DRIVETRAIN.LEFT_FRONT_CANCODER,
                        Constants.Drivetrain.LEFT_FRONT_STEER_OFFSET
                )),
                //right front module
                new SwerveModule(1, new Configurations.SwerveModuleConstants(
                        RobotMap.ROBOT.DRIVETRAIN.RIGHT_FRONT_DRIVE,
                        RobotMap.ROBOT.DRIVETRAIN.RIGHT_FRONT_ANGLE,
                        RobotMap.ROBOT.DRIVETRAIN.RIGHT_FRONT_CANCODER,
                        Constants.Drivetrain.RIGHT_FRONT_STEER_OFFSET
                )),
                //back left module
                new SwerveModule(2, new Configurations.SwerveModuleConstants(
                        RobotMap.ROBOT.DRIVETRAIN.LEFT_REAR_DRIVE,
                        RobotMap.ROBOT.DRIVETRAIN.LEFT_REAR_ANGLE,
                        RobotMap.ROBOT.DRIVETRAIN.LEFT_REAR_CANCODER,
                        Constants.Drivetrain.LEFT_REAR_STEER_OFFSET
                )),
                //back right module
                new SwerveModule(3, new Configurations.SwerveModuleConstants(
                        RobotMap.ROBOT.DRIVETRAIN.RIGHT_REAR_DRIVE,
                        RobotMap.ROBOT.DRIVETRAIN.RIGHT_REAR_ANGLE,
                        RobotMap.ROBOT.DRIVETRAIN.RIGHT_REAR_CANCODER,
                        Constants.Drivetrain.RIGHT_REAR_STEER_OFFSET
                ))

        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Drivetrain.swerveKinematics, getYaw(), getSwervePositions());
        this.thetaGains = Constants.Auto.kAutoThetaGains;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                Constants.Drivetrain.swerveKinematics.toSwerveModuleStates(
                        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation,
                                getYaw()
                        )
                                : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Drivetrain.maxSpeed);

        for(SwerveModule mod : swerveModules){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Drivetrain.maxSpeed);

        for(SwerveModule mod : swerveModules){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getSwervePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : swerveModules){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

/*    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveModules){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }*/

    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        return (Constants.Sensors.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getSwervePositions());

        for(SwerveModule mod : swerveModules){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }

    public static SwerveSubsystem getInstance() {
        if (instance==null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }

    public void resetGyroAndOdometry(boolean _input) {
        if (_input) {
            this.gyro.reset();
            _input = false;
        }
    }

    public boolean navXIsConnected() {
        return gyro.isConnected();
    }


    public void calibrateGyroscope() {
        this.gyro.calibrate();
    }

    public void zeroGyroscope() {
        this.gyro.zeroYaw();
        this.swerveOdometry.resetPosition(gyro.getRotation2d(), this.getSwervePositions(), new Pose2d());

    }

    public SwerveModulePosition[] getSwervePositions() {
        return new SwerveModulePosition[]{this.swerveModules[0].getPosition(),this.swerveModules[1].getPosition(),this.swerveModules[2].getPosition(),this.swerveModules[3].getPosition()};
    }

    public Gains getThetaGains() {
        return this.thetaGains;
    }

    public void setCurrentRobotPose(Pose2d pose) {
        this.swerveOdometry.resetPosition(this.gyro.getRotation2d(), this.getSwervePositions(),pose);
    }

    public Pose2d getCurrentRobotPose() {
        return this.swerveOdometry.getPoseMeters();
    }

    public Rotation2d getGyroscopeRotation() {
        // // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }
}