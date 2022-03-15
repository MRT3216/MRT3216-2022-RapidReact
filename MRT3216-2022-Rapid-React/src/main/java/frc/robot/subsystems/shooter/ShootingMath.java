package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

public class ShootingMath {

    // Camera Constants
    private final static double kCameraHeight = 0.721; // The vertical distance from the center of the camera to the
                                                       // ground.
    private final static double kCameraViewAngle = Units.degreesToRadians(50); // Angle (rads) of the center of POV of
                                                                               // the camera from horizontal.
    private final static double kCameraOffsetFromFrame = 0.04; // The horizontal distance from the center of the camera
                                                               // to the front of the robot.

    // Shooter Constants
    private final static double kShooterOffsetFromFrame = 0.634; // The horizontal distance from the front of the robot
                                                                 // to the shooter.
    private final static double kShooterHeight = 0.688; // The vertical distance from the ground to the shooter.

    // Goal Constants
    private final static double kTargetHeight = 2.642; // The vertical distance from the vision tape to the ground.
    private final static double kTargetGoalHorizontalOffest = 0.61; // The horiztonal distance from the vison tape to
                                                                    // the center of the goal;

    // Projectile Constant
    private final static double kMaxProjectileHeight = 3.25; // The maximum height from the ground of the projectile.
    private final static double kMinPorjectileHoriztonalVelocity = 0.75; // The minimum horizontal velocity of the
                                                                         // projectile. (m/s)
    private final static double kAccelDueToGravity = 9.8; // The acceleration due to gravity (m/s^s)
    private final static double kTargetHeightFromCamera = kTargetHeight - kCameraHeight; // The vertical distance from
                                                                                         // the camera to the shooter.

    private final static double kInitVerticalVelocity = Math
            .sqrt(2 * kAccelDueToGravity * (kMaxProjectileHeight - kShooterHeight)); // The initial vertical velocity of
                                                                                     // the projectile.

    /*
     * Takes in the angle (rads) of the vision target from the camera's center of
     * POV
     * and returns the distance to the center of the goal.
     */
    public static double getHorizontalGoalDistance(double cameraAngle) {
        // The horizontal distance from the center of the camera to the vision tape.
        double cameraXDist = kTargetHeightFromCamera / Math.tan(kCameraViewAngle + cameraAngle);

        // The horizontal distance from the front of the robot to the center of the
        // goal.
        double robotXDistToGoal = (cameraXDist - kCameraOffsetFromFrame) + kTargetGoalHorizontalOffest;

        // The horizontal distance from the shooter to the center of the goal.
        return robotXDistToGoal + kShooterOffsetFromFrame;
    }

    public static double getInitHoriztonalVelocity(double cameraAngle) {
        return getHorizontalGoalDistance(cameraAngle) + kMinPorjectileHoriztonalVelocity;
    }

    public static double getInitialVelocity(double cameraAngle) {
        return Math.sqrt(Math.pow(getHorizontalGoalDistance(cameraAngle), 2) + Math.pow(kInitVerticalVelocity, 2));
    }

    public static double getProjectileLaunchAngle(double cameraAngle) {
        return Math.atan(getHorizontalGoalDistance(cameraAngle) / kInitVerticalVelocity);
    }
}
