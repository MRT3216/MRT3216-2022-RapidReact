package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class LimelightSubsystem extends SubsystemBase implements Loggable{
	private static LimelightSubsystem instance;
	private final NetworkTable limelightNT;

	private LimelightSubsystem() {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		this.limelightNT = table;
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
		return limelightNT.getEntry("tx").getDouble(0.0);
	}

	/**
	 * Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
	 * 
	 * @return vertical offset to target (-20.5 degrees to 20.5 degrees)
	 */
	public double getVerticalOffset() {
		return limelightNT.getEntry("ty").getDouble(0.0);
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
	// 3 = forece on
	public boolean setLEDMode(int mode) {
		try {
			limelightNT.getEntry("ledMode").setNumber(mode);
			return true;
		} catch (Exception e) {
			return false;
		}
	}

	// modes:
	// 0 = vision processor
	// 1 = driver camera
	public boolean setCamMode(int mode) {
		try {
			limelightNT.getEntry("camMode").setNumber(mode);
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
	public boolean setStream(int stream) {
		try {
			limelightNT.getEntry("stream").setNumber(stream);
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
