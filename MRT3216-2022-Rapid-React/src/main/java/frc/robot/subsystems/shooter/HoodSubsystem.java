/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;

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

    /**
     * Creates a new Hopper.
     */
    public HoodSubsystem() {
        // aimServo = new Servo(SHOOTER.HOOD_ENCODER_PWM_PORT);
        encoder = new DutyCycleEncoder(SHOOTER.HOOD_ENCODER_PWM_PORT);
        hoodMotor = new CANSparkMax(SHOOTER.HOOD_MOTOR, Constants.kBrusheless);
        hoodMotor.setInverted(false);
        hoodMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public boolean isAimed() {
        return Math.abs(getAngle() - this.targetAngle) <= Hood.kMaxHoodError;
    }

    public void setAngle(double position) {
        if (position > Hood.hoodForwardLimit && position < Hood.hoodReverseLimit) {
            // goto position
            if (getAngle() < position) {
                runMotor(false);
            } else if (getAngle() > position) {
                runMotor(true);
            }
        } else if (position > Hood.hoodForwardLimit) {
            runMotor(true);
        } else if (position < Hood.hoodReverseLimit) {
            runMotor(false);
        } else {
            this.hoodMotor.stopMotor();
        }
    }

    public void runMotor(boolean reverse) {
        if (reverse) {
            this.hoodMotor.set(-Hood.hoodSpeed);
        } else {
            this.hoodMotor.set(Hood.hoodSpeed);
        }
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

    public double getPosition() {
        return getAngle();
        // return aimServo.getPosition();
    }

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
}