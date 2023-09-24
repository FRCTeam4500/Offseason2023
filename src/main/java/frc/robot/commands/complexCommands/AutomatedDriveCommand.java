package frc.robot.commands.complexCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.utilities.HelperMethods;

public class AutomatedDriveCommand extends CommandBase {

	private SwerveDrive swerve;
	private Pose2d[] targetPoses;
	private Pose2d currentTargetPose;
	private Transform2d relativeRobotPose;
	private Pose2d relativeOrigin;
	private double translationalThreshold;
	private double rotationalThreshold;
	private double timeThreshold;
	private double originalRobotZero;
	private double currentRobotZero;
	private double timeCorrect;
	private int poseCounter;
	private AutoDriveMode mode;
	private PIDController forwardVelocityController = new PIDController(
		5,
		0,
		0
	);
	private PIDController sidewaysVelocityController = new PIDController(
		5,
		0,
		0
	);
	private PIDController rotationalVelocityController = new PIDController(
		2,
		0,
		0
	);

	/**
	 * Drives the robot to a target field-centric position
	 * @param swerve the drivetrain subsystem
	 * @param mode the reference frame of the target positions.
	 * @param translationalThreshold the tolerance, in meters, for if the robot considers itself to be in the same spot as the target pose
	 * @param rotationalThreshold the tolerance, in meters, for if the robot considers itself to be facing the same direction as the target pose
	 * @param timeThreshold the time, in seconds, that the robot must be at the target pose before it is considered finished
	 * @param targetPose2ds the positions the robot should go to, in the order the robot should go to them
	 */
	public AutomatedDriveCommand(
		SwerveDrive swerve,
		AutoDriveMode mode,
		double translationalThreshold,
		double rotationalThreshold,
		double timeThreshold,
		Pose2d... targetPose2ds
	) {
		this.swerve = swerve;
		this.targetPoses = targetPose2ds;
		this.translationalThreshold = translationalThreshold;
		this.rotationalThreshold = rotationalThreshold;
		this.timeThreshold = timeThreshold;
		this.mode = mode;
		originalRobotZero = swerve.getCurrentZero();
		swerve.resetRobotAngle();
		currentRobotZero = swerve.getCurrentZero();
		addRequirements(swerve);
	}

	public enum AutoDriveMode {
		/** The target positions are relative to the robot */
		kRelative,
		/** The target positions are relative to the field */
		kAbsolute,
	}

	@Override
	public void initialize() {
		timeCorrect = 0;
		poseCounter = 0;
		forwardVelocityController.reset();
		sidewaysVelocityController.reset();
		rotationalVelocityController.reset();
		relativeOrigin = swerve.getRobotPose();
		// This next line just converts all the target positions to be relative to the robot, if they aren't already
		if (mode == AutoDriveMode.kAbsolute) {
			for (int i = 0; i < targetPoses.length; i++) {
				targetPoses[i] = targetPoses[i].relativeTo(relativeOrigin);
			}
		}
	}

	@Override
	public void execute() {
		relativeRobotPose = swerve.getRobotPose().minus(relativeOrigin);
		currentTargetPose = targetPoses[poseCounter];
		double forwardVelocity = forwardVelocityController.calculate(
			relativeRobotPose.getX(),
			currentTargetPose.getX()
		);
		double sidewaysVelocity = sidewaysVelocityController.calculate(
			relativeRobotPose.getY(),
			currentTargetPose.getY()
		);
		double rotationalVelocity = rotationalVelocityController.calculate(
			relativeRobotPose.getRotation().getRadians() % (2 * Math.PI),
			currentTargetPose.getRotation().getRadians()
		);
		swerve.driveFieldCentric(
			forwardVelocity,
			sidewaysVelocity,
			rotationalVelocity
		);
		Pose2d robotPoseRel = new Pose2d(
			relativeRobotPose.getTranslation(),
			relativeRobotPose.getRotation()
		);
		if (HelperMethods.isClose(
				currentTargetPose,
				robotPoseRel,
				translationalThreshold,
				rotationalThreshold
			)
		) {
			timeCorrect++;
		} else {
			timeCorrect = 0;
		}

		if (timeCorrect / 50 >= timeThreshold) { // the / 50 part is because there are 50 scheduler ticks a second
			poseCounter++;
			timeCorrect = 0;
			forwardVelocityController.reset();
			sidewaysVelocityController.reset();
			rotationalVelocityController.reset();
		}
	}

	@Override
	public boolean isFinished() {
		return poseCounter > targetPoses.length - 1;
	}

	@Override
	public void end(boolean interrupted) {
		swerve.resetRobotAngle(swerve.getRobotAngle() + currentRobotZero - originalRobotZero);
		swerve.driveFieldCentric(0, 0, 0);
	}
}
