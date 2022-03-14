/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Shooter.Hood;
import frc.robot.settings.RobotMap.ROBOT.SHOOTER;
import io.github.oblarg.oblog.Loggable;

public class HoodSubsystem extends SubsystemBase implements Loggable {
    private static HoodSubsystem instance;
    private final DutyCycleEncoder encoder;
    private final CANSparkMax hoodMotor;
    private double targetAngle;
    private double manualErrorAdjustment;
    private SparkMaxPIDController hoodPIDController;


    /**
     * Creates a new Hopper.
     */
    public HoodSubsystem() {
        // aimServo = new Servo(SHOOTER.HOOD_ENCODER_PWM_PORT);
        encoder = new DutyCycleEncoder(SHOOTER.HOOD_ENCODER_PWM_PORT);
        hoodMotor = new CANSparkMax(SHOOTER.HOOD_MOTOR, Constants.kBrusheless);
        hoodMotor.restoreFactoryDefaults();
        hoodMotor.setInverted(false);
        hoodMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        hoodMotor.enableVoltageCompensation(12);
        hoodMotor.burnFlash();
        hoodPIDController = hoodMotor.getPIDController();
        hoodPIDController.setOutputRange(-1,1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public boolean isAimed() {
        return Math.abs(getAngle() - this.targetAngle) <= Hood.kMaxHoodError;
    }

    public void setAngle(double position) {
        //todo: pid code goes here
    }

    /**
     * Finds the motor's current angle. Note this doesn't reflect the true position
     * of the hood, but the last position the hood was commanded to. It will not
     * account for the time it takes for the mechanism to get to its destination
     * position. Therefore this method SHOULD NOT be used to qualify a command has
     * completed moving the hood.
     * 
     * @return the angle (degrees) the motor was last commanded to.
     */
    public double getAngle() {
        // return aimServo.getAngle();
        return encoder.getAbsolutePosition();
    }


    //todo: determing what here is necessary and fix it

    public void setHoodFromPitch(double pitch) {
        double servoAngle = calculateHoodFromPitch(pitch);
        servoAngle -= this.manualErrorAdjustment;
        // aimServo.setAngle(servoAngle);
    }

    private double calculateHoodFromPitch(double x) {
        x += Constants.Shooter.Hood.hoodErrorAdjustment;
        return -0.0374 * Math.pow(x, 2) - 0.5155 * x + 119.28;
    }

    public double getHoodErrorAdjustment() {
        return Constants.Shooter.Hood.hoodErrorAdjustment;
    }

    public void setHoodErrorAdjustment(double value) {
        Constants.Shooter.Hood.hoodErrorAdjustment = value;
    }

    public double getHoodManualAdjustment() {
        return this.manualErrorAdjustment;
    }

    public void setHoodManualAdjustment(double value) {
        this.manualErrorAdjustment = value;
    }

    public static HoodSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new HoodSubsystem();
        }
        return instance;
    }

    public void setP(int P) {this.hoodPIDController.setP(P);}
    public void setI(int I) {this.hoodPIDController.setI(I);}
    public void setD(int D) {this.hoodPIDController.setD(D);}
}