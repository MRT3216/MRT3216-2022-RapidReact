package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.util.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.settings.*;
import io.github.oblarg.oblog.*;

public class ColorSensorSubsystem extends SubsystemBase implements Loggable {

    private ColorSensorV3 sensor;
    private muxSubsystem mux = new muxSubsystem();
    private int sensIndex;

    public ColorSensorSubsystem(int sensIndex) {
        this.sensIndex = sensIndex;
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

    public boolean isRed() {
        return inRange() && getColor().red > getColor().blue; // if in range && red > blue
    }

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