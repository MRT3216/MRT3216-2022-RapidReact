package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.LimeLight;
import frc.robot.settings.Constants.LimeLight.CameraMode;
import frc.robot.settings.Constants.LimeLight.CameraStream;
import frc.robot.settings.Constants.LimeLight.LEDMode;
import frc.robot.settings.Constants.Projectile;
import io.github.oblarg.oblog.Loggable;

public class LimelightSubsystem extends SubsystemBase implements Loggable {
	private static LimelightSubsystem instance;
	private final NetworkTable limelightNT;

	private LimelightSubsystem() {
		NetworkTable table = NetworkTableInstance.getDefault().getTable(LimeLight.NTtable);
		this.limelightNT = table;
		System.out.println("Trying to turn off LED with value: " + LimeLight.LEDMode.OFF.ordinal());
		this.setLEDMode(LimeLight.LEDMode.OFF);
	}

	/*
	 *  Returns the horizontal distance to the center of the goal in meters.
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

	// ---------- getters ----------
	/**
	 * Returns whether the limelight has any valid targets (0 or 1)
	 * 
	 * @return whether the limelight has any valid targets
	 */
	public boolean hasTarget() {
		return limelightNT.getEntry("tv").getDouble(0) == 1 ? true : false;
	}

	/**
	 * Returns Horizontal Offset From Crosshair To Target (-27 degrees to 27
	 * degrees)
	 * 
	 * @return horizontal offset to target (-27 degrees to 27 degrees)
	 */
	public double getHorizontalOffset() {
		return -limelightNT.getEntry("ty").getDouble(0.0);
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
