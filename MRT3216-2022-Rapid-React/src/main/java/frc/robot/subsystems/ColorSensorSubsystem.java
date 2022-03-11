package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.settings.RobotMap.ROBOT.SENSORS;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ColorSensorSubsystem extends SubsystemBase implements Loggable {

    private ColorSensorV3 sensor;
    private muxSubsystem mux = new muxSubsystem();
    private int sensIndex;

    public ColorSensorSubsystem() {
        this.sensIndex = SENSORS.COLOR;
        this.mux.setIndex(this.sensIndex);
        sensor = new ColorSensorV3(mux.getPort());
    }

    private Color getColor() {
        // Weirdly, this "getColor" is actually returning a stabilized color value (by a couple ms), not
        // the raw input, as with
        // .getRawColor - this is perfect for our use though, as we're just looking for
        // the "average" color
        mux.setIndex(this.sensIndex);
        return sensor.getColor();
    }

    @Log.BooleanBox(name = "Red Detected", rowIndex = 0, columnIndex = 0)
    public boolean isRed() {
        System.out.println("Range: " + getProximity() + " value=" + getColor());
        return inRange() && getColor().red > getColor().blue; // if in range && red > blue
    }

    @Log.BooleanBox(name = "Blue Detected", rowIndex = 0, columnIndex = 1)
    public boolean isBlue() {
        return inRange() && !isRed(); // if in range && not red
    }

    private double getProximity() {
        mux.setIndex(this.sensIndex);
        return sensor.getProximity(); // get proximity
    }

    private boolean inRange() {
        return getProximity() > Constants.Sensors.ColorRange; // prox > wanted range
    }

}