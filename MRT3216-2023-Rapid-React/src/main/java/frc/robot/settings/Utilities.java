package frc.robot.settings;

public class Utilities {
    public static double convertUnitsPer100msToRPM(double unitsPer100ms, double unitsPerRotation) {
        return unitsPer100ms * 600 / unitsPerRotation;
    }

    public static double convertRPMsToUnitsPer100ms(double rpm, double unitsPerRotation) {
        return rpm * unitsPerRotation / 600.0;
    }
}