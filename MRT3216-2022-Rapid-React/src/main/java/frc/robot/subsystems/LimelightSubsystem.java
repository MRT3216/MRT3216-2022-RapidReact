package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.LimeLight;
import frc.robot.settings.Constants.LimeLight.CameraMode;
import frc.robot.settings.Constants.LimeLight.CameraStream;
import frc.robot.settings.Constants.LimeLight.LEDMode;
import frc.robot.settings.Constants.Projectile;

public class LimelightSubsystem extends SubsystemBase {
	private static LimelightSubsystem instance;
	private final NetworkTable limelightNT;
	private final LinearFilter horizontalFilter;

	private LimelightSubsystem() {
		NetworkTable table = NetworkTableInstance.getDefault().getTable(LimeLight.NTtable);
		this.limelightNT = table;
		this.horizontalFilter = LinearFilter.movingAverage(5);
		this.setLEDMode(LimeLight.LEDMode.OFF);
	}

	/*
	 * Returns the horizontal distance to the center of the goal in meters.
	 */
	public double getHorizontalGoalDistance() {
		double cameraAngle = Units.degreesToRadians(this.getVerticalOffset());
		// The horizontal distance from the center of the camera to the vision tape.
		double cameraXDist = Projectile.kTargetHeightFromCamera / Math.tan(Projectile.kCameraViewAngle + cameraAngle);

		// The horizontal distance from the front of the robot to the center of the
		// goal.
		double robotXDistToGoal = (cameraXDist - Projectile.kCameraOffsetFromFrame)
				+ Projectile.kTargetGoalHorizontalOffest;

		// The horizontal distance from the shooter to the center of the goal.
		return robotXDistToGoal + Projectile.kShooterOffsetFromFrame;
	}

	public double getInitHoriztonalVelocity() {
		double goalFeet = (Units.metersToFeet(getHorizontalGoalDistance()) * 3 / 4 + 1);
		return Units.feetToMeters(goalFeet);
	}

	public double getInitVerticalVelocity() {
		double goalDistanceFeet = Units.metersToFeet(this.getHorizontalGoalDistance());
		double goalHeightFeet = 10 + goalDistanceFeet / 8;
		double goalHeightMeters = Units.feetToMeters(goalHeightFeet);
		double goalVelMeters = Math
				.sqrt(2 * Projectile.kAccelDueToGravity * (goalHeightMeters - Projectile.kShooterHeight));
		return goalVelMeters;
	}

	public double getInitialRPM() {
		return 165.83 * getHorizontalGoalDistance() + 1971.2;
	}

	// ---------- getters ----------
	/**
	 * Returns whether the limelight has any valid targets (0 or 1)
	 * 
	 * @return whether the limelight has any valid targets
	 */
	public boolean hasTarget() {
		return limelightNT.getEntry("tv").getDouble(0) == 1;
	}

	public double getLatency() {
		return limelightNT.getEntry("tl").getDouble(0);
	}

	/**
	 * Returns Horizontal Offset From Crosshair To Target (-27 degrees to 27
	 * degrees)
	 * 
	 * @return horizontal offset to target (-27 degrees to 27 degrees)
	 */
	public double getHorizontalOffset() {
		return horizontalFilter.calculate(limelightNT.getEntry("ty").getDouble(0.0));
	}

	/**
	 * Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
	 * 
	 * @return vertical offset to target (-20.5 degrees to 20.5 degrees)
	 */
	public double getVerticalOffset() {
		return limelightNT.getEntry("tx").getDouble(0.0);
	}

	/**
	 * Returns Target Area (0% of image to 100% of image)
	 * 
	 * @return target area (0% of image to 100% of image)
	 */
	public double getTargetArea() {
		return limelightNT.getEntry("ta").getDouble(0.0);
	}

	// ---------- setters ----------

	// modes:
	// 0 = use the LED Mode set in the current pipeline
	// 1 = force off
	// 2 = force blink
	// 3 = force on
	public boolean setLEDMode(LEDMode mode) {
		try {
			limelightNT.getEntry("ledMode").setNumber(mode.ordinal());
			return true;
		} catch (Exception e) {
			return false;
		}
	}

	public void setLEDModeByInt(int mode) {
		LEDMode newMode = LEDMode.OFF;

		switch (mode) {
			case 0:
				newMode = LEDMode.PIPELINE;
				break;
			case 1:
				newMode = LEDMode.OFF;
				break;
			case 2:
				newMode = LEDMode.BLINK;
				break;
			case 3:
				newMode = LEDMode.ON;
				break;
		}

		this.setLEDMode(newMode);
	}

	// modes:
	// 0 = vision processor
	// 1 = driver camera
	public boolean setCamMode(CameraMode mode) {
		try {
			limelightNT.getEntry("camMode").setNumber(mode.ordinal());
			return true;
		} catch (Exception e) {
			return false;
		}
	}

	// pipeline: Select pipeline 0..9
	public boolean setPipeline(int pipeline) {
		try {
			limelightNT.getEntry("pipeline").setNumber(pipeline);
			return true;
		} catch (Exception e) {
			return false;
		}
	}

	// Set stream:
	// 0 = Standard - Side-by-side streams if a webcam is attached to Limelight
	// 1 = PiP Main - The secondary camera stream is placed in the lower-right
	// corner of the primary camera stream
	// 2 = PiP Secondary - The primary camera stream is placed in the lower-right
	// corner of the secondary camera stream
	public boolean setStream(CameraStream stream) {
		try {
			limelightNT.getEntry("stream").setNumber(stream.ordinal());
			return true;
		} catch (Exception e) {
			return false;
		}
	}

	public void setStreamByInt(int mode) {
		CameraStream newMode = CameraStream.PiPMain;

		switch (mode) {
			case 0:
				newMode = CameraStream.Standard;
				break;
			case 1:
				newMode = CameraStream.PiPMain;
				break;
			case 2:
				newMode = CameraStream.PiPSecondary;
				break;
		}

		this.setStream(newMode);
	}

	// Set snapshot:
	// 0 = Stop taking snapshots
	// 1 = Take two snapshots per second
	public boolean setSnapshot(int snapshot) {
		try {
			limelightNT.getEntry("snapshot").setNumber(snapshot);
			return true;
		} catch (Exception e) {
			return false;
		}
	}

	public static LimelightSubsystem getInstance() {
		if (instance == null) {
			// if instance is null, initialize
			instance = new LimelightSubsystem();
		}
		return instance;
	}
}
