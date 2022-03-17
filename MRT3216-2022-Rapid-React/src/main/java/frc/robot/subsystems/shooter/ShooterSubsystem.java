package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Configurations;
import frc.robot.settings.Constants.Projectile;
import frc.robot.settings.Constants.Shooter.Flywheel;
import frc.robot.settings.RobotMap.ROBOT.SHOOTER;
import frc.robot.settings.Utilities;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem instance;
    private TalonFX flywheelMotor;
    private double shootingVelocityUnitsPer100ms;
    private double ejectVelocityUnitsPer100ms;

    private ShooterSubsystem() {
        flywheelMotor = new TalonFX(SHOOTER.FLYWHEEL_MOTOR);
        flywheelMotor.configFactoryDefault();
        flywheelMotor.configAllSettings(Configurations.getInstance().getFlywheelMotorConfiguration());

        flywheelMotor.setInverted(Flywheel.FLYWHEEL_MOTOR_INVERTED);
        flywheelMotor.setNeutralMode(NeutralMode.Coast);
        flywheelMotor.enableVoltageCompensation(true);

        /* Set status frame periods to ensure we don't have stale data */
        flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20, Flywheel.kTimeoutMs);
        flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, Flywheel.kTimeoutMs);
        flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, Flywheel.kTimeoutMs);

        this.shootingVelocityUnitsPer100ms = Utilities.convertRPMsToUnitsPer100ms(Flywheel.shootingRPM,
                Flywheel.kSensorUnitsPerRotation);
        this.ejectVelocityUnitsPer100ms = Utilities.convertRPMsToUnitsPer100ms(Flywheel.ejectRPM,
                Flywheel.kSensorUnitsPerRotation);

        this.zeroSensors();
    }

    @Override
    public void periodic() {
    }

    public void spinToSpeed(boolean forward) {
        // Velocity closed loop without feed forward (not sure if this is enough)
        if (forward) {
            flywheelMotor.set(TalonFXControlMode.Velocity, this.shootingVelocityUnitsPer100ms);
        } else if (!forward) {
            flywheelMotor.set(TalonFXControlMode.PercentOutput, -1 * .25);
        }
    }

    public void eject() {
        flywheelMotor.set(TalonFXControlMode.Velocity, this.ejectVelocityUnitsPer100ms);
    }

    public double getRPM() {
        return Utilities.convertUnitsPer100msToRPM(flywheelMotor.getSelectedSensorVelocity(),
                Flywheel.kSensorUnitsPerRotation);
    }

    public void stopShooter() {
        if (flywheelMotor != null) {
            flywheelMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        }
    }

    /* Zero all sensors on Talons */
    public void zeroSensors() {
        flywheelMotor.getSensorCollection().setIntegratedSensorPosition(0, Flywheel.kTimeoutMs);
    }

    public void setShootingRPM(double rpm) {
        this.shootingVelocityUnitsPer100ms = Utilities.convertRPMsToUnitsPer100ms(rpm,
                Flywheel.kSensorUnitsPerRotation);

    }

    /*
     * Takes in the angle (rads) of the vision target from the camera's center of
     * POV
     * and returns the distance to the center of the goal.
     */
    public static double getHorizontalGoalDistance(double cameraAngle) {
        // The horizontal distance from the center of the camera to the vision tape.
        double cameraXDist = Projectile.kTargetHeightFromCamera / Math.tan(Projectile.kCameraViewAngle + cameraAngle);

        // The horizontal distance from the front of the robot to the center of the
        // goal.
        double robotXDistToGoal = (cameraXDist - Projectile.kCameraOffsetFromFrame) + Projectile.kTargetGoalHorizontalOffest;

        // The horizontal distance from the shooter to the center of the goal.
        return robotXDistToGoal + Projectile.kShooterOffsetFromFrame;
    }

    public static double getInitHoriztonalVelocity(double cameraAngle) {
        return getHorizontalGoalDistance(cameraAngle) + Projectile.kMinPorjectileHoriztonalVelocity;
    }

    public static double getInitialVelocity(double cameraAngle) {
        return Math.sqrt(Math.pow(getHorizontalGoalDistance(cameraAngle), 2) + Math.pow(Projectile.kInitVerticalVelocity, 2));
    }

    public static double getProjectileLaunchAngle(double cameraAngle) {
        return Math.atan(getHorizontalGoalDistance(cameraAngle) / Projectile.kInitVerticalVelocity);
    }


    public double getShootingRPM() {
        return Utilities.convertUnitsPer100msToRPM(this.shootingVelocityUnitsPer100ms,
                Flywheel.kSensorUnitsPerRotation);

    }

    public void setEjectRPM(double rpm) {
        this.ejectVelocityUnitsPer100ms = Utilities.convertRPMsToUnitsPer100ms(rpm, Flywheel.kSensorUnitsPerRotation);
    }

    public void setPValue(double p) {
        this.flywheelMotor.config_kP(Flywheel.kSlotIdx, p);
    }

    public void setIValue(double i) {
        this.flywheelMotor.config_kP(Flywheel.kSlotIdx, i);
    }

    public void setDValue(double d) {
        this.flywheelMotor.config_kP(Flywheel.kSlotIdx, d);
    }

    public void setFValue(double f) {
        this.flywheelMotor.config_kF(Flywheel.kSlotIdx, f);
    }

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new ShooterSubsystem();
        }
        return instance;
    }
}