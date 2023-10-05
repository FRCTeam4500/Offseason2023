package frc.robot.commands.complexCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DriveController;
import frc.robot.Constants.EnumConstants.ControlMode;
import frc.robot.Constants.EnumConstants.VisionTarget;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

/**
 * A swerve command with support for two swerve control modes:
 * <p>
 * Field-Centric:
 * Robot moves relative to the field's axes.
 * When pushing the joystick forward, the robot moves down the field, no matter which way it is facing
 * (Actually, it moves in whatever direction is zeroed to, this just assumes that the gyro is zeroed down the field)
 * <p>
 * Robot-Centric:
 * The robot moves relative to itself.
 * When pushing the joystick foward, the robot moves in whatever direction it is facing.
 * For our purposes, the front of the robot is the intake side.
 */
public class SwerveDriveCommand extends CommandBase {
	private SwerveDrive swerve;
	private Vision vision;
	private CommandXboxController controller;

	private ControlMode controlMode;
	private ControlMode previousControlMode;

	private VisionTarget visionTarget;
	public SlewRateLimiter xLimiter = new SlewRateLimiter(1.75);
	public SlewRateLimiter yLimiter = new SlewRateLimiter(1.75);
	public SlewRateLimiter zLimiter = new SlewRateLimiter(1.4);

	private PIDController pid;
	private boolean doSlew;

	private double xSens;
	private double ySens;
	private double zSens;
	
	private double xSpeed;
	private double ySpeed;
	private double zSpeed;

	private double targetAngle;
	private int timesBalanced;

	private static SwerveDriveCommand instance = null;

	public static synchronized SwerveDriveCommand getInstance(DriveController xbox) {
		if (instance == null) {
			instance = new SwerveDriveCommand(xbox);
		}
		return instance;
	}

	public static synchronized SwerveDriveCommand getInstance() {
		return instance;
	}

	private SwerveDriveCommand(DriveController xbox) {
		swerve = SwerveDrive.getInstance();
		vision = Vision.getInstance();
		controller = xbox;
		pid = new PIDController(1, 0, 0);
		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		setControlMode(ControlMode.FieldCentric);
		targetAngle = 0;
		visionTarget = VisionTarget.AprilTag;
		timesBalanced = 0;
		fastSpeed();
	}

	@Override
	public void execute() {
		checkForHoldAngle();
		convertControllerSpeeds();
		double offsetDegrees = getTargetOffsetDegrees();
		switch (getControlMode()) {
			case FieldCentric:
				moveFieldCentric(xSpeed, ySpeed, zSpeed);
				break;
			case RobotCentric:
				moveRobotCentric(xSpeed, ySpeed, zSpeed);
				break;
			case HoldAngle:
				if (Math.abs(controller.getRightX()) > 0.1) {
					setControlMode(previousControlMode);
				} else {
					moveAngleCentric(xSpeed, ySpeed);
				}
				break;
			case AimToTarget:
				moveRobotCentric(xSpeed, 0, pid.calculate(offsetDegrees)/10);
				break;
			case AlignToTarget:
				moveRobotCentric(xSpeed, pid.calculate(offsetDegrees)/10, 0);
				break;
			case Balance:
				pid.setTolerance(3);
				moveRobotCentric(pid.calculate(Math.toDegrees(swerve.getGyro().getPitch()))/20, 0, 0);
				checkForBalance();
				break;
		}
	}
	
	private void checkForBalance() {
		if (pid.atSetpoint()) {
			timesBalanced++;
		} else {
			timesBalanced = 0;
		}
		if (timesBalanced > 50) {
			switchControlMode();
		}
	}

	private double getTargetOffsetDegrees() {
		vision.setPipeline(visionTarget.limelightId, visionTarget.pipeline);
		double offsetDegrees = vision.getHorizontalAngleOffset(visionTarget.limelightId);
		if (!vision.hasValidTargets(visionTarget.limelightId)) {
			offsetDegrees = 0;
		}
		return offsetDegrees;
	}

	private void checkForHoldAngle() {
		if (controller.getRightY() > 0.5) {
			setControlMode(ControlMode.HoldAngle);
			targetAngle = 180;
		}
		if (controller.getRightY() < -0.5) {
			setControlMode(ControlMode.HoldAngle);
			targetAngle = 0;
		}
	}

	private void convertControllerSpeeds() {
		if(doSlew) {
			xSpeed = -xLimiter.calculate(controller.getLeftX()) * xSens;
			ySpeed = -yLimiter.calculate(controller.getLeftY()) * ySens;
		} else {
			xSpeed = -controller.getLeftX() * xSens;
			ySpeed = -controller.getLeftY() * ySens;
		}
			zSpeed = -controller.getRightX() * zSens;
	}
	
	private void moveFieldCentric(double x, double y, double w) {
		swerve.driveFieldCentric(y, x, w);
	}

	private void moveRobotCentric(double x, double y, double w) {
		swerve.driveRobotCentric(y, x, w);
	}

	private void moveAngleCentric(double xSpeed, double ySpeed) {
		double wSpeed =
			4 *
			pid.calculate(
				swerve.getRobotAngle(),
				Math.toRadians(targetAngle)
			);
		moveFieldCentric(xSpeed, ySpeed, wSpeed);
	}

	public void fastSpeed() {
		xSens = 4;
		ySens = 4;
		zSens = 3.5;
		doSlew = true;
		MessagingSystem.getInstance().addMessage("Swerve -> Robot Speed -> Fast");
	}

	public void slowSpeed() {
		xSens = .8;
		ySens = .8;
		zSens = .5;
		doSlew = false;
		MessagingSystem.getInstance().addMessage("Swerve -> Robot Speed -> Slow");
	}

	public void setTargetAngle(double angle) {
		targetAngle = angle;
	}

	public void setVisionTarget(VisionTarget newVisionTarget) {
		visionTarget = newVisionTarget;
	}

	public void switchControlMode() {
		if (controlMode == ControlMode.FieldCentric) {
			setControlMode(ControlMode.RobotCentric);
		} else {
			setControlMode(ControlMode.FieldCentric);
		}
	}

	public void setControlMode(ControlMode newControlMode) {
		if (controlMode == ControlMode.RobotCentric || 
			controlMode == ControlMode.FieldCentric) {
			previousControlMode = controlMode;
		}
		pid.reset();
		controlMode = newControlMode;
		MessagingSystem.getInstance().addMessage("Swerve -> Control Mode -> " + newControlMode.name());
	}

	public ControlMode getControlMode() {
		return controlMode;
	}

	public double getTargetAngle() {
		return targetAngle;
	}

	public void initSendable(SendableBuilder builder) {
		builder.addStringProperty("Drive Mode: ", () -> getControlMode().name(), null);
		builder.addDoubleProperty("Target Angle: ", () -> targetAngle, null);
	}
}
