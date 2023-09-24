/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.hardware;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Limelight {

	private NetworkTable table;

	public Limelight(String limelightName) {
		table = NetworkTableInstance.getDefault().getTable(limelightName);
		table.getEntry("getpipe").setNumber(0);
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

	/**
	 * Pose of the robot in Field Coordinates
	 * @return Pose3d x,y,z pitch,yaw,roll
	 */
	public Pose3d getRobotPoseToField() {
		double[] raw = table.getEntry("botpose").getDoubleArray(new double[6]);
		return new Pose3d(
			raw[0],
			raw[1],
			raw[2],
			new Rotation3d(raw[3], raw[4], raw[5])
		);
	}

	/**
	 * Pose of the robot in Field Coordinates relative to Alliance
	 * @return
	 */
	public Pose3d getRobotPoseToAlliance(Alliance alliance) {
		double[] raw;
		String allianceString;
		if (alliance.name() != null) {
			allianceString = alliance.name();
		} else {
			return null;
		}
		raw = table.getEntry(allianceString).getDoubleArray(new double[6]);
		return new Pose3d(
			raw[0],
			raw[1],
			raw[2],
			new Rotation3d(raw[3], raw[4], raw[5])
		);
	}

	/**
	 * Pose of the robot in terms of the April Tag coordinate system
	 * @return Transform3D x,y,z pitch,yaw,roll
	 */
	public Pose3d getRobotPoseToTarget() {
		double[] raw = table
			.getEntry("botpose_targetspace")
			.getDoubleArray(new double[6]);
		return new Pose3d(
			raw[0],
			raw[1],
			raw[2],
			new Rotation3d(raw[3], raw[4], raw[5])
		);
	}

	/**
	 * Pose of the Target April Tag in terms of the Camera coordinate system
	 * @return Transform3D x,y,z pitch,yaw,roll
	 */
	public Pose3d getTargetPoseToCamera() {
		double[] raw = table
			.getEntry("targetpose_cameraspace")
			.getDoubleArray(new double[6]);
		return new Pose3d(
			raw[0],
			raw[1],
			raw[2],
			new Rotation3d(raw[3], raw[4], raw[5])
		);
	}

	/**
	 * Pose of the Target April Tag in terms of Robot coordinate system
	 * @return Transform3D x,y,z pitch,yaw,roll
	 */
	public Pose3d getTargetPoseToRobot() {
		double[] raw = table
			.getEntry("targetpose_robotspace")
			.getDoubleArray(new double[6]);
		return new Pose3d(
			raw[0],
			raw[1],
			raw[2],
			new Rotation3d(raw[3], raw[4], raw[5])
		);
	}

	/**
	 * Pose of the camera in terms of the Target coordinate system
	 * @return Transform3D x,y,z pitch,yaw,roll
	 */
	public Pose3d getCameraPoseToTarget() {
		double[] raw = table
			.getEntry("camerapose_targetspace")
			.getDoubleArray(new double[6]);
		return new Pose3d(
			raw[0],
			raw[1],
			raw[2],
			new Rotation3d(raw[3], raw[4], raw[5])
		);
	}

	public int getTargetTagId() {
		return (int) table.getEntry("tid").getInteger(0);
	}
}
