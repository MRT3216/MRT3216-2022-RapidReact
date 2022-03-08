package frc.robot.OI;

public class OIUtils {
    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    public static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, Constants.OI.kJoystickDeadband);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }

     public static double expo(double value) {

     }
}
