package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ColorSensorSubsystem extends SubsystemBase implements Loggable {
    private static ColorSensorSubsystem instance;
    private ColorSensorV3 sensor;

    public ColorSensorSubsystem() {
        sensor = new ColorSensorV3(I2C.Port.kMXP);
    }

    private Color getColor() {
        // Weirdly, this "getColor" is actually returning a stabilized color value (by a
        // couple ms), not
        // the raw input, as with
        // .getRawColor - this is perfect for our use though, as we're just looking for
        // the "average" color
        return sensor.getColor();
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