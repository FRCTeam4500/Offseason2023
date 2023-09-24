package frc.robot.commands.complexCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DriveController;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;

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
	private CommandXboxController controller;

	public ControlMode controlMode;
	private ControlMode previousControlMode;

	public SlewRateLimiter xLimiter = new SlewRateLimiter(1);
	public SlewRateLimiter yLimiter = new SlewRateLimiter(1);
	public SlewRateLimiter zLimiter = new SlewRateLimiter(1.4);

	private PIDController angleController;
	private boolean doSlew;

	private double xSens;
	private double ySens;
	private double zSens;
	
	private double xSpeed;
	private double ySpeed;
	private double zSpeed;

	public double targetAngle = 0;

	public SwerveDriveCommand() {
		swerve = SwerveDrive.getInstance();
		controller = DriveController.getInstance();
		controlMode = ControlMode.FieldCentric; //default control mode is field-centric
		angleController = new PIDController(1, 0, 0);
		angleController.enableContinuousInput(-Math.PI, Math.PI);
		fastSpeed();
		addRequirements(swerve);
	}

	@Override
	public void execute() {
		if (controller.getRightY() > 0.5) {
			controlMode = ControlMode.AngleCentric;
			targetAngle = 180;
		}
		if (controller.getRightY() < -0.5) {
			controlMode = ControlMode.AngleCentric;
			targetAngle = 0;
		}
		if (controlMode != ControlMode.AngleCentric) {
			previousControlMode = controlMode;
		}

		if(doSlew) {
			xSpeed = -xLimiter.calculate(controller.getLeftX()) * xSens;
			ySpeed = -yLimiter.calculate(controller.getLeftY()) * ySens;
			zSpeed = -zLimiter.calculate(controller.getRightX()) * zSens;
		} else {
			xSpeed = -controller.getLeftX() * xSens;
			ySpeed = -controller.getLeftY() * ySens;
			zSpeed = -controller.getRightX() * zSens;
		}

		switch (controlMode) {
			case FieldCentric:
				moveFieldCentric(xSpeed, ySpeed, zSpeed);
				break;
			case RobotCentric:
				moveRobotCentric(xSpeed, ySpeed, zSpeed);
				break;
			case AngleCentric:
				if (Math.abs(controller.getRightX()) > 0.1) {
					controlMode = previousControlMode;
				} else {
					moveAngleCentric(xSpeed, ySpeed);
				}
				break;
		}
	}

	private void moveFieldCentric(double x, double y, double w) {
		swerve.driveFieldCentric(y, x, w);
	}

	private void moveRobotCentric(double x, double y, double w) {
		swerve.driveRobotCentric(y, x, w);
	}

	private void moveAngleCentric(double xSpeed, double ySpeed) {
		double wSpeed =
			2 *
			angleController.calculate(
				swerve.getRobotAngle(),
				Math.toRadians(targetAngle)
			);
		moveFieldCentric(xSpeed, ySpeed, wSpeed);
	}

	public enum ControlMode {
		FieldCentric,
		RobotCentric,
		AngleCentric,
	}

	public void fastSpeed() {
		xSens = 4;
		ySens = 4;
		zSens = 3.5;
		doSlew = true;
		MessagingSystem.getInstance().addMessage("Swerve -> Robot Speed -> Fast");
	}

	public void midSpeed() {
		xSens = 2;
		ySens = 2;
		zSens = 1.75;
		doSlew = false;
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

	/**
	 * Switches between RobotCentric and FieldCentric
	 */
	public void switchControlMode() {
		if (controlMode == ControlMode.FieldCentric) {
			controlMode = ControlMode.RobotCentric;
			MessagingSystem.getInstance().addMessage("Swerve -> Control Mode -> Robot Centric");
		} else {
			controlMode = ControlMode.FieldCentric;
			MessagingSystem.getInstance().addMessage("Swerve -> Control Mode -> Field Centric");
		}
	}

	public void initSendable(SendableBuilder builder) {
		builder.addStringProperty("Drive Mode", () -> controlMode.name(), null);
		builder.addDoubleProperty("Target Angle: ", () -> targetAngle, null);
	}
}
