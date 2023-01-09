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
import frc.robot.settings.RobotMap;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] swerveModules;
    public AHRS gyro;

    SwerveSubsystem instance;

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

        swerveOdometry = new SwerveDriveOdometry(Constants.Drivetrain.swerveKinematics, getYaw(), getModulePositions());
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
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : swerveModules){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveModules){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        return (Constants.Sensors.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());

        for(SwerveModule mod : swerveModules){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }

    public SwerveSubsystem getInstance() {
        if (instance==null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }
}