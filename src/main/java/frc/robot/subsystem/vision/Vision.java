package frc.robot.subsystem.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.component.LimelightVisionComponent;
import frc.robot.component.LimelightVisionComponent.CameraMode;

public class Vision extends SubsystemBase implements VisionInterface{

	private static Vision instanceVision;
	private LimelightVisionComponent[] limelights = new LimelightVisionComponent[2];

	private VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

	private Vision() {
		limelights[0] = new LimelightVisionComponent("limelight-hehehe");
		// limelights[1] = new LimelightVisionComponent("limelight-haha");
	}

	public VisionInputsAutoLogged getInputs() {
		return inputs;
	}

	public void updateInputs(VisionInputs inputs) {
		inputs.horizontalAngleOffset = getHorizontalAngleOffset();
		inputs.robotPose = getRobotPose();
	}

	/**
	 * Gets the instance of the vision. If the instance doesn't exist, it creates it
	 * @return the instance of the swerve drive
	 */
	public static synchronized Vision getInstance() {
		if (instanceVision == null) {
			instanceVision = new Vision();
		}
		return instanceVision;
	}

	public double getHorizontalAngleOffset() {
		return getHorizontalAngleOffset(0);
	}

	public double getHorizontalAngleOffset(int limelightId) {
		return limelights[limelightId].getHorizontalOffsetFromCrosshair();
	}

	public double getVerticalAngleOffset() {
		return getVerticalAngleOffset(0);
	}

	public double getVerticalAngleOffset(int limelightId) {
		return limelights[limelightId].getVerticalOffsetFromCrosshair();
	}

	public double getTakenArea() {
		return getTakenArea(0);
	}

	public double getTakenArea(int limelightId) {
		return limelights[limelightId].getTargetArea();
	}

	public double getSkew() {
		return getSkew(0);
	}

	public double getSkew(int limelightId) {
		return limelights[limelightId].getSkew();
	}

	public Pose2d getRobotPose() {
		return getRobotPose(0);
	}

	public Pose2d getRobotPose(int limelightId) {
		return limelights[limelightId].getRobotPoseToAlliance(DriverStation.getAlliance()).toPose2d();
	}

	public Pose2d getTargetPose() {
		return getTargetPose(0);
	}

	public Pose2d getTargetPose(int limelightId) {
		return limelights[limelightId].getTargetPoseToRobot().toPose2d();
	}

	public boolean hasValidTargets() {
		return hasValidTargets(0);
	}

	public boolean hasValidTargets(int limelightId) {
		return limelights[limelightId].hasValidTargets();
	}

	public void setPipeline(int pipeline) {
		setPipeline(0, pipeline);;
	}

	public void setPipeline(int limelightId, int pipeline) {
		limelights[limelightId].setPipeline(pipeline);
	}

	public void setCameraMode(CameraMode mode) {
		setCameraMode(0, mode);;
	}

	public void setCameraMode(int limelightId, CameraMode mode) {
		limelights[limelightId].setCameraMode(mode);
	}

	public LimelightVisionComponent getLimelight() {
		return getLimelight(0);
	}

	public LimelightVisionComponent getLimelight(int limelightId) {
		return limelights[limelightId];
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty("Hehehe: Valid Targets", () -> hasValidTargets(), null);
		builder.addDoubleProperty("Hehehe: Horizontal Offset (Degrees)", () -> Units.radiansToDegrees(getHorizontalAngleOffset()), null);
		// builder.addBooleanProperty("Haha: Valid Targets", () -> hasValidTargets(1), null);
		// builder.addDoubleProperty("Haha: Horizontal Offset (Degrees)", () -> Units.radiansToDegrees(getHorizontalAngleOffset(1)), null);
	}
}