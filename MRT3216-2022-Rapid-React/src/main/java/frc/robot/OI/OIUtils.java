package frc.robot.OI;

import frc.robot.settings.Constants.OI;

public class OIUtils {
    public static double modifyAxis(double value, double expo) {
        // Deadband
        value = deadband(value, OI.kJoystickDeadband);

        value = expo(value, expo);
        return value;
    }

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

    private static double expo(double value, double expo) {
        double adjValue = (1 - ((100 - expo) / 100)) * Math.pow(value, 3)
                + (value * ((100 - expo) / 100));

        System.out.println("Value: " + value);
        System.out.println("Adj Value: " + adjValue);
        return adjValue;
    }
}
