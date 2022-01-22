package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.settings.RobotMap;

public class ColorSensorSubsystem extends SubsystemBase {
    private ColorSensorV3 colorSens = new ColorSensorV3(RobotMap.ROBOT.SENSORS.colorSensor);

    public ColorSensorSubsystem() {
    }

    private Color getColor() {
        // Weirdly, this "getColor" is actually returning a stabilized color value, not
        // the raw input, as with
        // .getRawColor - this is perfect for our use though, as we're just looking for
        // the "average" color
        return colorSens.getColor();
    }

    public boolean isRed() {
        return inRange() && getColor().red > getColor().blue; // if in range && red > blue
    }

    public boolean isBlue() {
        return inRange() && !isRed(); // if in range && not red
    }

    private double getProximity() {
        return colorSens.getProximity(); // get proximity
    }

    private boolean inRange() {
        return getProximity() > Constants.Sensors.ColorRange; // prox > wanted range
    }

}
