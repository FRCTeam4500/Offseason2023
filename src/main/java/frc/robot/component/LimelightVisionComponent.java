/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.component;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utility.Transform3D;

/**
 * An {@link VisionComponent} wrapper for a Limelight vision processor.
 */
public class LimelightVisionComponent {

	private NetworkTable table;

	/**
	 * Creates a new VisionComponent component, which is essentially a wrapper
	 * around the networktable entries modified by a Limelight.
	 */
	public LimelightVisionComponent() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
	}

	public enum CameraMode {
		VisionProcessor,
		DriverCamera,
	}

	public boolean hasValidTargets() {
		double value = getEntry("tv");
		if (value == 1) {
			return true;
		} else {
			return false;
		}
	}

	public double getHorizontalOffsetFromCrosshair() {
		return Math.toRadians(getEntry("tx"));
	}

	public double getVerticalOffsetFromCrosshair() {
		return Math.toRadians(getEntry("ty"));
	}

	public double getTargetArea() {
		return getEntry("ta");
	}

	public double getSkew() {
		double rawDegrees = getEntry("ts");
		double adjustedDegrees;
		if (Math.abs(rawDegrees) < 45) {
			adjustedDegrees = -rawDegrees;
		} else {
			adjustedDegrees = -(90 + rawDegrees);
		}
		return Math.toRadians(adjustedDegrees);
	}

	public void setCameraMode(CameraMode mode) {
		if (mode == CameraMode.DriverCamera) {
			setEntry("camMode", 1);
		} else if (mode == CameraMode.VisionProcessor) {
			setEntry("camMode", 0);
		} // Maybe add exception for bad entry
	}

	public void setPipeline(int index) {
		setEntry("pipeline", index);
	}

	private double getEntry(String key) {
		return table.getEntry(key).getDouble(0);
	}

	private void setEntry(String key, Number value) {
		table.getEntry(key).setNumber(value);
	}

	@Deprecated
	private double[] getCameraTranslationEntry() {
		return table.getEntry("camtran").getDoubleArray(new double[6]);
	}

	@Deprecated
	public Transform3D getCameraTransform() {
		double[] translation = getCameraTranslationEntry();
		return new Transform3D(
			translation[0],
			translation[1],
			translation[2],
			translation[3],
			translation[4],
			translation[5]
		);
	}

	/**
	 * Pose of the robot in Field Coordinates
	 * @return Transform3D x,y,z pitch,yaw,roll
	 */
	public Transform3D getRobotPoseToField() {
		double[] raw = table.getEntry("botpose").getDoubleArray(new double[6]);
		return new Transform3D(raw[0], raw[1], raw[2], raw[4], raw[5], raw[3]);
	}

	/**
	 * Pose of the robot in terms of the April Tag coordinate system
	 * @return Transform3D x,y,z pitch,yaw,roll
	 */
	public Transform3D getRobotPoseToTarget() {
		double[] raw = table
			.getEntry("botpose_targetspace")
			.getDoubleArray(new double[6]);
		return new Transform3D(raw[0], raw[1], raw[2], raw[4], raw[5], raw[3]);
	}

	/**
	 * Pose of the Target April Tag in terms of the Camera coordinate system
	 * @return Transform3D x,y,z pitch,yaw,roll
	 */
	public Transform3D getTargetPoseToCamera() {
		double[] raw = table
			.getEntry("targetpose_cameraspace")
			.getDoubleArray(new double[6]);
		return new Transform3D(raw[0], raw[1], raw[2], raw[4], raw[5], raw[3]);
	}

	/**
	 * Pose of the Target April Tag in terms of Robot coordinate system
	 * @return Transform3D x,y,z pitch,yaw,roll
	 */
	public Transform3D getTargetPoseToRobot() {
		double[] raw = table
			.getEntry("targetpose_robotspace")
			.getDoubleArray(new double[6]);
		return new Transform3D(raw[0], raw[1], raw[2], raw[4], raw[5], raw[3]);
	}

	/**
	 * Pose of the camera in terms of the Target coordinate system
	 * @return Transform3D x,y,z pitch,yaw,roll
	 */
	public Transform3D getCameraPoseToTarget() {
		double[] raw = table
			.getEntry("camerapose_targetspace")
			.getDoubleArray(new double[6]);
		return new Transform3D(raw[0], raw[1], raw[2], raw[4], raw[5], raw[3]);
	}

	/**
	 * Target ID
	 * @return id of target :)
	 */
	public int getTargetId() {
		return (int) table.getEntry("tid").getInteger(0);
	}

	/**
	 * Transform3D to Pose2d
	 */
	public Pose2d getPoseFromTransform3D(Transform3D transform) {
		return new Pose2d(
			new Translation2d(transform.getX(), transform.getY()),
			new Rotation2d(transform.getYaw())
		);
	}
}
