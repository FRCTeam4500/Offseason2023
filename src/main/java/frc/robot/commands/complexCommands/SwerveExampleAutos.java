package frc.robot.commands.complexCommands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auton;
import frc.robot.subsystem.swerve.SwerveExample;
import java.util.HashMap;
import java.util.List;

public class SwerveExampleAutos {

	public class AutoBalanceCommand extends CommandBase {

		private final SwerveExample swerveSubsystem;
		private final PIDController controller;

		public AutoBalanceCommand(SwerveExample swerveSubsystem) {
			this.swerveSubsystem = swerveSubsystem;
			controller = new PIDController(1.0, 0.0, 0.0);
			controller.setTolerance(1);
			controller.setSetpoint(0.0);
			// each subsystem used by the command must be passed into the
			// addRequirements() method (which takes a vararg of Subsystem)
			addRequirements(this.swerveSubsystem);
		}

		/**
		 * The initial subroutine of a command.  Called once when the command is initially scheduled.
		 */
		@Override
		public void initialize() {}

		/**
		 * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
		 * until {@link #isFinished()}) returns true.)
		 */
		@Override
		public void execute() {
			SmartDashboard.putBoolean("At Tolerance", controller.atSetpoint());

			double translationVal = MathUtil.clamp(
				controller.calculate(
					swerveSubsystem.getPitch().getDegrees(),
					0.0
				),
				-0.5,
				0.5
			);
			swerveSubsystem.drive(
				new Translation2d(translationVal, 0.0),
				0.0,
				true,
				false
			);
		}

		/**
		 * <p>
		 * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
		 * the scheduler will call its {@link #end(boolean)} method.
		 * </p><p>
		 * Returning false will result in the command never ending automatically. It may still be cancelled manually or
		 * interrupted by another command. Hard coding this command to always return true will result in the command executing
		 * once and finishing immediately. It is recommended to use *
		 * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
		 * </p>
		 *
		 * @return whether this command has finished.
		 */
		@Override
		public boolean isFinished() {
			return controller.atSetpoint();
		}

		/**
		 * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
		 * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
		 * up loose ends, like shutting off a motor that was being used in the command.
		 *
		 * @param interrupted whether the command was interrupted/canceled
		 */
		@Override
		public void end(boolean interrupted) {
			swerveSubsystem.lock();
		}
	}

	public final class Autos {

		/**
		 * April Tag field layout.
		 */
		private static AprilTagFieldLayout aprilTagField = null;

		private Autos() {
			throw new UnsupportedOperationException("This is a utility class!");
		}

		public static CommandBase driveAndSpin(SwerveExample swerve) {
			return Commands.sequence(
				new RepeatCommand(
					new InstantCommand(
						() ->
							swerve.drive(
								new Translation2d(1, 0),
								5,
								true,
								true
							),
						swerve
					)
				)
			);
		}

		/**
		 * Example static factory for an autonomous command.
		 */
		public static CommandBase exampleAuto(SwerveExample swerve) {
			boolean onTheFly = false; // Use the path defined in code or loaded from PathPlanner.
			PathPlannerTrajectory example;
			if (onTheFly) {
				// Simple path with holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
				example =
					PathPlanner.generatePath(
						new PathConstraints(4, 3),
						new PathPoint(
							new Translation2d(0, 0),
							Rotation2d.fromDegrees(0),
							Rotation2d.fromDegrees(0)
						),
						// position, heading(direction of travel), holonomic rotation
						new PathPoint(
							new Translation2d(3, 5),
							Rotation2d.fromDegrees(90),
							Rotation2d.fromDegrees(90)
						),
						// position, heading(direction of travel), holonomic rotation
						new PathPoint(
							new Translation2d(5, 5),
							Rotation2d.fromDegrees(0),
							Rotation2d.fromDegrees(0)
						)
						// position, heading(direction of travel), holonomic rotation
					);
			} else {
				List<PathPlannerTrajectory> example1 = PathPlanner.loadPathGroup(
					"SamplePath",
					new PathConstraints(4, 3)
				);
				// This is just an example event map. It would be better to have a constant, global event map
				// in your code that will be used by all path following commands.
				HashMap<String, Command> eventMap = new HashMap<>();
				eventMap.put("marker1", new PrintCommand("Passed marker 1"));

				// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want
				// to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
				SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
					swerve::getPose,
					// Pose2d supplier
					swerve::resetOdometry,
					// Pose2d consumer, used to reset odometry at the beginning of auto
					new PIDConstants(
						Auton.yAutoPID.p,
						Auton.yAutoPID.i,
						Auton.yAutoPID.d
					),
					// PID constants to correct for translation error (used to create the X and Y PID controllers)
					new PIDConstants(
						Auton.angleAutoPID.p,
						Auton.angleAutoPID.i,
						Auton.angleAutoPID.d
					),
					// PID constants to correct for rotation error (used to create the rotation controller)
					swerve::setChassisSpeeds,
					// Module states consumer used to output to the drive subsystem
					eventMap,
					false,
					// Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
					swerve
					// The drive subsystem. Used to properly set the requirements of path following commands
				);
				return Commands.sequence(autoBuilder.fullAuto(example1));
			}
			//    swerve.postTrajectory(example);
			return Commands.sequence(
				new FollowTrajectory(swerve, example, true)
			);
		}

		/**
		 * Create a {@link FollowTrajectory} command to go to the April Tag from the current position.
		 *
		 * @param swerve            Swerve drive subsystem.
		 * @param id                April Tag ID to go to.
		 * @param rotation          Rotation to go to.
		 * @param holonomicRotation Holonomic rotation to be at.
		 * @param offset            Offset from the April Tag.
		 * @return {@link FollowTrajectory} command. May return null if cannot load field.
		 */
		public static CommandBase driveToAprilTag(
			SwerveExample swerve,
			int id,
			Rotation2d rotation,
			Rotation2d holonomicRotation,
			Translation2d offset
		) {
			if (aprilTagField == null) {
				try {
					aprilTagField =
						AprilTagFields.kDefaultField.loadAprilTagLayoutField();
				} catch (Exception ignored) {
					return null;
				}
			}
			PathPlannerTrajectory path = PathPlanner.generatePath(
				new PathConstraints(4, 3),
				false,
				PathPoint.fromCurrentHolonomicState(
					swerve.getPose(),
					swerve.getRobotVelocity()
				),
				new PathPoint(
					aprilTagField
						.getTagPose(id)
						.get()
						.getTranslation()
						.toTranslation2d()
						.plus(offset),
					rotation,
					holonomicRotation
				)
			);
			return Commands.sequence(new FollowTrajectory(swerve, path, false));
		}

		public class FollowTrajectory extends SequentialCommandGroup {

			public FollowTrajectory(
				SwerveExample drivebase,
				PathPlannerTrajectory trajectory,
				boolean resetOdometry
			) {
				addRequirements(drivebase);

				if (resetOdometry) {
					drivebase.resetOdometry(
						trajectory.getInitialHolonomicPose()
					);
				}

				addCommands(
					new PPSwerveControllerCommand(
						trajectory,
						drivebase::getPose,
						Auton.xAutoPID.createPIDController(),
						Auton.yAutoPID.createPIDController(),
						Auton.angleAutoPID.createPIDController(),
						drivebase::setChassisSpeeds,
						drivebase
					)
				);
			}
		}
	}
}
