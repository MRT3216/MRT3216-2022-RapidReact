package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterStateMachine;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Ball;
import frc.robot.settings.RobotMap.ROBOT.SENSORS;

public class ColorSensorSubsystem extends SubsystemBase {
    private static ColorSensorSubsystem instance;
    private ColorSensorV3 sensor;

    private ColorSensorSubsystem() {
        sensor = new ColorSensorV3(SENSORS.COLOR_SENSOR);
    }

    @Override
    public void periodic() {
        ShooterStateMachine.getInstance().setLastBall(getCurrentBall());
    }

    public Ball getCurrentBall() {
        if (!this.inRange()) {
            return Ball.NONE;
        } else if (this.isAllianceBall()) {
            return Ball.ALLIANCE;
        } else {
            return Ball.OPPONENT;
        }
    }

    private Color getColor() {
        // Weirdly, this "getColor" is actually returning a stabilized color value (by a
        // couple ms), not
        // the raw input, as with
        // .getRawColor - this is perfect for our use though, as we're just looking for
        // the "average" color
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

    public boolean isOpponentBall() {
        Alliance currentAlliance = DriverStation.getAlliance();
        if (currentAlliance == Alliance.Blue) {
            return this.isRed();
        } else {
            return this.isBlue();
        }
    }

    public boolean isRed() {
        return inRange() && getColor().red > getColor().blue; // if in range && red > blue
    }

    public boolean isBlue() {
        return inRange() && !isRed(); // if in range && not red
    }

    private double getProximity() {
        return sensor.getProximity(); // get proximity
    }

    public boolean inRange() {
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