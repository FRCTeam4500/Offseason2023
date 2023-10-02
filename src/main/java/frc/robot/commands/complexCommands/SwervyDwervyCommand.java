package frc.robot.commands.complexCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.DriveController;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swervydwervy.Swerve;

public class SwervyDwervyCommand extends CommandBase {

	private Swerve swerve;
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

	public SwervyDwervyCommand(DriveController xbox) {
		swerve = Swerve.getInstance();
		controller = xbox;
		controlMode = ControlMode.FieldCentric; // default control mode is field-centric
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

		if (doSlew) {
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
		swerve.move(
			y * SwerveConstants.MAX_LINEAR_SPEED,
			x * SwerveConstants.MAX_LINEAR_SPEED,
			w * SwerveConstants.MAX_ROTATIONAL_SPEED,
			true
		); // Convert from Percents to m/s
	}

	private void moveRobotCentric(double x, double y, double w) {
		swerve.move(
			y * SwerveConstants.MAX_LINEAR_SPEED,
			x * SwerveConstants.MAX_LINEAR_SPEED,
			w * SwerveConstants.MAX_ROTATIONAL_SPEED,
			false
		);
	}

	private void moveAngleCentric(double xSpeed, double ySpeed) {
		double wSpeed =
			2 *
			angleController.calculate(
				swerve.getHeading().getRadians(),
				Math.toRadians(targetAngle)
			);
		moveFieldCentric(
			xSpeed * SwerveConstants.MAX_LINEAR_SPEED,
			ySpeed * SwerveConstants.MAX_LINEAR_SPEED,
			wSpeed
		);
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
		MessagingSystem
			.getInstance()
			.addMessage("Swerve -> Robot Speed -> Fast");
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
		MessagingSystem
			.getInstance()
			.addMessage("Swerve -> Robot Speed -> Slow");
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
			MessagingSystem
				.getInstance()
				.addMessage("Swerve -> Control Mode -> Robot Centric");
		} else {
			controlMode = ControlMode.FieldCentric;
			MessagingSystem
				.getInstance()
				.addMessage("Swerve -> Control Mode -> Field Centric");
		}
	}

	public void initSendable(SendableBuilder builder) {
		builder.addStringProperty("Drive Mode", () -> controlMode.name(), null);
		builder.addDoubleProperty("Target Angle: ", () -> targetAngle, null);
	}

	/** Applies rate limit to joystick input. */
	public class RateLimit {

		private final double rateLimit;
		private double lastLimit;

		public RateLimit(double rateLimit) {
			this.rateLimit = rateLimit;
		}

		/**
		 * Return the joystick input adjusted to a rate limit.
		 *
		 * @param joystickInput joystick axis position
		 * @return the joystick axis position after rate limiting
		 */
		public double apply(double joystickInput) {
			double y;
			if (Math.abs(joystickInput - lastLimit) > rateLimit) {
				y =
					lastLimit +
					Math.copySign(rateLimit, joystickInput - lastLimit);
			} else {
				y = joystickInput;
			}

			lastLimit = y;
			return y;
		}
	}

	/** Applies exponential scaling and deadband to joystick inputs */
	public class ExpoScale {

		private final double deadband;
		private final double scale;
		private final double offset;

		public ExpoScale(double deadband, double scale) {
			this.deadband = deadband;
			this.scale = scale;
			offset =
				1.0 /
				(
					scale *
					Math.pow(1 - deadband, 3) +
					(1 - scale) *
					(1 - deadband)
				);
		}

		/**
		 * Return the joystick axis position input adjusted on an exponential scale with
		 * deadband
		 * adjustment.
		 *
		 * @param input the joystick axis position
		 * @return the adjusted input value, range is -1.0 to 1.0
		 */
		public double apply(double input) {
			double y;

			if (Math.abs(input) < deadband) {
				return 0;
			}

			y = input > 0 ? input - deadband : input + deadband;
			return (scale * Math.pow(y, 3) + (1 - scale) * y) * offset;
		}
	}
}
