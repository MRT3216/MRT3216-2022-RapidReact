package frc.robot.settings;

/**
 * Class that organizes gains used when assigning values to slots
 */
public class Gains {
    public double kP;
    public double kI;
    public double kD;
    public double kF;
    public int kIzone;
    public double kPeakOutput;

    public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput) {
        kP = _kP;
        kI = _kI;
        kD = _kD;
        kF = _kF;
        kIzone = _kIzone;
        kPeakOutput = _kPeakOutput;
    }

    public Gains(double _kP, double _kI, double _kD) {
        this(_kP, _kI, _kD, 0, 0, 0);
    }
}