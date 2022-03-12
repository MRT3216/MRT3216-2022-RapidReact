/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Shooter.Hood;
import frc.robot.settings.RobotMap.ROBOT.SHOOTER;

public class OldHoodSubsystem extends SubsystemBase {
    private final Servo aimServo;
    private double targetAngle;

    private double manualErrorAdjustment;

    /**
     * Creates a new Hopper.
     */
    public OldHoodSubsystem() {
        aimServo = new Servo(SHOOTER.HOOD_ENCODER_PWM_PORT);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public boolean isAimed() {
        return Math.abs(getAngle() - this.targetAngle) <= Hood.kMaxHoodError;
    }

    public void setAngle(double position) {
        aimServo.setAngle(position);
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
        return aimServo.getAngle();
    }

    public double getPosition() {
        return aimServo.getPosition();
    }

    public void setHoodFromPitch(double pitch) {
        double servoAngle = calculateHoodFromPitch(pitch);
        servoAngle -= this.manualErrorAdjustment;
        aimServo.setAngle(servoAngle);
    }

    private double calculateHoodFromPitch(double x) {
        x += Hood.hoodErrorAdjustment;
        return -0.0374 * Math.pow(x, 2) - 0.5155 * x + 119.28;
    }

    public double getHoodErrorAdjustment() {
        return Hood.hoodErrorAdjustment;
    }

    public void setHoodErrorAdjustment(double value) {
        Hood.hoodErrorAdjustment = value;
    }

    public double getHoodManualAdjustment() {
        return this.manualErrorAdjustment;
    }

    public void setHoodManualAdjustment(double value) {
        this.manualErrorAdjustment = value;
    }
}