package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Limelight;
import frc.robot.hardware.Limelight.CameraMode;

public class Vision extends SubsystemBase implements VisionInterface{

	private static Vision instanceVision;
	private Limelight[] limelights = new Limelight[2];

	private VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

	private Vision() {
		limelights[0] = new Limelight("limelight-hehehe"); // Limelight 3, used for april tags
		limelights[1] = new Limelight("limelight-haha"); // Limelight 2, used for game pieces
		setPipeline(0, 0);
		setPipeline(1, 0);
	}

	public VisionInputsAutoLogged getInputs() {
		return inputs;
	}

	public void updateInputs(VisionInputs inputs) {
		inputs.horizontalAngleOffset = getHorizontalAngleOffset(0);
		inputs.robotPose = getRobotPose(0);
	}

	public static synchronized Vision getInstance() {
		if (instanceVision == null) {
			instanceVision = new Vision();
		}
		return instanceVision;
	}

	public double getHorizontalAngleOffset(int limelightId) {
		return limelights[limelightId].getHorizontalOffsetFromCrosshair();
	}

	public double getVerticalAngleOffset(int limelightId) {
		return limelights[limelightId].getVerticalOffsetFromCrosshair();
	}

	public double getTakenArea(int limelightId) {
		return limelights[limelightId].getTargetArea();
	}

	public double getSkew(int limelightId) {
		return limelights[limelightId].getSkew();
	}

	public Pose2d getRobotPose(int limelightId) {
		return limelights[limelightId].getRobotPoseToAlliance(DriverStation.getAlliance()).toPose2d();
	}

	public Pose2d getRelativeTargetPose(int limelightId) {
		return limelights[limelightId].getTargetPoseToRobot().toPose2d();
	}

	public boolean hasValidTargets(int limelightId) {
		return limelights[limelightId].hasValidTargets();
	}

	public void setPipeline(int limelightId, int pipeline) {
		limelights[limelightId].setPipeline(pipeline);
	}

	public void setCameraMode(int limelightId, CameraMode mode) {
		limelights[limelightId].setCameraMode(mode);
	}

	public Limelight getLimelight(int limelightId) {
		return limelights[limelightId];
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty("Hehehe: Valid Targets", () -> hasValidTargets(0), null);
		builder.addDoubleProperty("Hehehe: Horizontal Offset (Degrees)", () -> Units.radiansToDegrees(getHorizontalAngleOffset(0)), null);
		builder.addDoubleProperty("Hehehe: Target Area (%)", () -> getTakenArea(0), null);
	}
}