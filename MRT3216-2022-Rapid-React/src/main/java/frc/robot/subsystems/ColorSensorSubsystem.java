package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.settings.RobotMap.ROBOT.SENSORS;

public class ColorSensorSubsystem extends SubsystemBase {
    private static ColorSensorSubsystem instance;
    private ColorSensorV3 sensor;

    private ColorSensorSubsystem() {
        sensor = new ColorSensorV3(SENSORS.COLOR_SENSOR);
    }

    private Color getColor() {
        // Weirdly, this "getColor" is actually returning a stabilized color value (by a
        // couple ms), not
        // the raw input, as with
        // .getRawColor - this is perfect for our use though, as we're just looking for
        // the "average" color
        System.out.println("Color: " + sensor.getColor());
        return sensor.getColor();
    }

    public boolean isAllianceBall() {
        Alliance currentAlliance = DriverStation.getAlliance();
        if (currentAlliance == Alliance.Blue) {
            return this.isBlue();
        } else {
            return this.isRed();
        }
    }

    @Log.BooleanBox(name = "Red Detected", rowIndex = 0, columnIndex = 0)
    public boolean isRed() {
        return inRange() && getColor().red > getColor().blue; // if in range && red > blue
    }

    @Log.BooleanBox(name = "Blue Detected", rowIndex = 0, columnIndex = 1)
    public boolean isBlue() {
        return inRange() && !isRed(); // if in range && not red
    }

    private double getProximity() {
        return sensor.getProximity(); // get proximity
    }

    private boolean inRange() {
        return getProximity() > Constants.Sensors.ColorRange; // prox > wanted range
    }

    public static ColorSensorSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new ColorSensorSubsystem();
        }
        return instance;
    }
}