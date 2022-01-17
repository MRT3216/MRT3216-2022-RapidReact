package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.settings.RobotMap;


public class ColorSensorSubsystem extends SubsystemBase {
    private ColorSensorV3 colorSens = new ColorSensorV3(RobotMap.ROBOT.SENSORS.colorSensor);
    public ColorSensorSubsystem () {
    }

    private Color getColor () {
        //  Weirdly, this "getColor" is actually returning a stabilized color value, not the raw input, as with
        //  .getRawColor - this is perfect for our use though, as we're just looking for the "average" color
        return colorSens.getColor();
    }

    public boolean isRed() {
        // if (inRange() && getColor().red > getColor().blue) { // if in the defined range (defined in settings.constants and red > blue)
        //     return true;
        // }
        // else {
        //     return false;
        // }
        return inRange() && getColor().red > getColor().blue;
    }

    public boolean isBlue() {
        // if (inRange()) {
        //     return !isRed();
        // }
        // else {
        //     return false;
        // }
        return inRange() && !isRed();
    }

    private double getProximity() {
        return colorSens.getProximity();
    }

    private boolean inRange() {
        if (getProximity() > Constants.Sensors.ColorRange) {
            return true;
        }
        else {
            return false;
        }
    }

}
