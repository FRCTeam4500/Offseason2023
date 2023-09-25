package frc.robot.commands.complexCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystem.swerve.SwerveExample;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class SwerveExampleCommand {

	public class AbsoluteDrive extends CommandBase {

		private final SwerveExample swerve;
		private final DoubleSupplier vX, vY;
		private final DoubleSupplier headingHorizontal, headingVertical;
		private final boolean isOpenLoop;

		/**
		 * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
		 * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
		 * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
		 * will rotate to.
		 *
		 * @param swerve            The swerve drivebase subsystem.
		 * @param vX                DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1
		 *                          to 1 with deadband already accounted for.  Positive X is away from the alliance wall.
		 * @param vY                DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1
		 *                          to 1 with deadband already accounted for.  Positive Y is towards the left wall when
		 *                          looking through the driver station glass.
		 * @param headingHorizontal DoubleSupplier that supplies the horizontal component of the robot's heading angle. In the
		 *                          robot coordinate system, this is along the same axis as vY. Should range from -1 to 1 with
		 *                          no deadband.  Positive is towards the left wall when looking through the driver station
		 *                          glass.
		 * @param headingVertical   DoubleSupplier that supplies the vertical component of the robot's heading angle.  In the
		 *                          robot coordinate system, this is along the same axis as vX.  Should range from -1 to 1
		 *                          with no deadband. Positive is away from the alliance wall.
		 */
		public AbsoluteDrive(
			SwerveExample swerve,
			DoubleSupplier vX,
			DoubleSupplier vY,
			DoubleSupplier headingHorizontal,
			DoubleSupplier headingVertical,
			boolean isOpenLoop
		) {
			this.swerve = swerve;
			this.vX = vX;
			this.vY = vY;
			this.headingHorizontal = headingHorizontal;
			this.headingVertical = headingVertical;
			this.isOpenLoop = isOpenLoop;

			addRequirements(swerve);
		}

		@Override
		public void initialize() {}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			// Get the desired chassis speeds based on a 2 joystick module.

			ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(
				vX.getAsDouble(),
				vY.getAsDouble(),
				headingHorizontal.getAsDouble(),
				headingVertical.getAsDouble()
			);

			// Limit velocity to prevent tippy
			Translation2d translation = SwerveController.getTranslation2d(
				desiredSpeeds
			);
			translation =
				SwerveMath.limitVelocity(
					translation,
					swerve.getFieldVelocity(),
					swerve.getPose(),
					Constants.LOOP_TIME,
					Constants.ROBOT_MASS,
					List.of(Constants.CHASSIS),
					swerve.getSwerveDriveConfiguration()
				);
			SmartDashboard.putNumber("LimitedTranslation", translation.getX());
			SmartDashboard.putString("Translation", translation.toString());

			// Make the robot move
			swerve.drive(
				translation,
				desiredSpeeds.omegaRadiansPerSecond,
				true,
				isOpenLoop
			);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public class AbsoluteFieldDrive extends CommandBase {

		private final SwerveExample swerve;
		private final DoubleSupplier vX, vY, heading;
		private final boolean isOpenLoop;

		/**
		 * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
		 * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
		 * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
		 * will rotate to.
		 *
		 * @param swerve  The swerve drivebase subsystem.
		 * @param vX      DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1 with
		 *                deadband already accounted for.  Positive X is away from the alliance wall.
		 * @param vY      DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1 with
		 *                deadband already accounted for.  Positive Y is towards the left wall when looking through the driver
		 *                station glass.
		 * @param heading DoubleSupplier that supplies the robot's heading angle.
		 */
		public AbsoluteFieldDrive(
			SwerveExample swerve,
			DoubleSupplier vX,
			DoubleSupplier vY,
			DoubleSupplier heading,
			boolean isOpenLoop
		) {
			this.swerve = swerve;
			this.vX = vX;
			this.vY = vY;
			this.heading = heading;
			this.isOpenLoop = isOpenLoop;

			addRequirements(swerve);
		}

		@Override
		public void initialize() {}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			// Get the desired chassis speeds based on a 2 joystick module.

			ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(
				vX.getAsDouble(),
				vY.getAsDouble(),
				new Rotation2d(heading.getAsDouble() * Math.PI)
			);

			// Limit velocity to prevent tippy
			Translation2d translation = SwerveController.getTranslation2d(
				desiredSpeeds
			);
			translation =
				SwerveMath.limitVelocity(
					translation,
					swerve.getFieldVelocity(),
					swerve.getPose(),
					Constants.LOOP_TIME,
					Constants.ROBOT_MASS,
					List.of(Constants.CHASSIS),
					swerve.getSwerveDriveConfiguration()
				);
			SmartDashboard.putNumber("LimitedTranslation", translation.getX());
			SmartDashboard.putString("Translation", translation.toString());

			// Make the robot move
			swerve.drive(
				translation,
				desiredSpeeds.omegaRadiansPerSecond,
				true,
				isOpenLoop
			);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public class TeleopDrive extends CommandBase {

		private final SwerveExample swerve;
		private final DoubleSupplier vX;
		private final DoubleSupplier vY;
		private final DoubleSupplier omega;
		private final BooleanSupplier driveMode;
		private final boolean isOpenLoop;
		private final SwerveController controller;
		private final Timer timer = new Timer();
		private final boolean headingCorrection;
		private double angle = 0;
		private double lastTime = 0;

		/**
		 * Creates a new ExampleCommand.
		 *
		 * @param swerve The subsystem used by this command.
		 */
		public TeleopDrive(
			SwerveExample swerve,
			DoubleSupplier vX,
			DoubleSupplier vY,
			DoubleSupplier omega,
			BooleanSupplier driveMode,
			boolean isOpenLoop,
			boolean headingCorrection
		) {
			this.swerve = swerve;
			this.vX = vX;
			this.vY = vY;
			this.omega = omega;
			this.driveMode = driveMode;
			this.isOpenLoop = isOpenLoop;
			this.controller = swerve.getSwerveController();
			this.headingCorrection = headingCorrection;
			if (headingCorrection) {
				timer.start();
			}
			// Use addRequirements() here to declare subsystem dependencies.
			addRequirements(swerve);
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			if (headingCorrection) {
				lastTime = timer.get();
			}
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			double xVelocity = Math.pow(vX.getAsDouble(), 3);
			double yVelocity = Math.pow(vY.getAsDouble(), 3);
			double angVelocity = Math.pow(omega.getAsDouble(), 3);
			SmartDashboard.putNumber("vX", xVelocity);
			SmartDashboard.putNumber("vY", yVelocity);
			SmartDashboard.putNumber("omega", angVelocity);
			if (headingCorrection) {
				// Estimate the desired angle in radians.
				angle +=
					(angVelocity * (timer.get() - lastTime)) *
					controller.config.maxAngularVelocity;
				// Get the desired ChassisSpeeds given the desired angle and current angle.
				ChassisSpeeds correctedChassisSpeeds = controller.getTargetSpeeds(
					xVelocity,
					yVelocity,
					angle,
					swerve.getHeading().getRadians()
				);
				// Drive using given data points.
				swerve.drive(
					SwerveController.getTranslation2d(correctedChassisSpeeds),
					correctedChassisSpeeds.omegaRadiansPerSecond,
					driveMode.getAsBoolean(),
					isOpenLoop
				);
				lastTime = timer.get();
			} else {
				// Drive using raw values.
				swerve.drive(
					new Translation2d(
						xVelocity * controller.config.maxSpeed,
						yVelocity * controller.config.maxSpeed
					),
					angVelocity * controller.config.maxAngularVelocity,
					driveMode.getAsBoolean(),
					isOpenLoop
				);
			}
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}
}
