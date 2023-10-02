package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;

public class Constants {

	public static class JoystickConstants {

		// Joystick ports
		public static final int DRIVER_PORT = 2;
		public static final int OPERATOR_PORT = 1;

		// Control stick map
		public static final int PLACE = 1;

		public static final int CUBE_INTAKE = 12;
		public static final int CONE_INTAKE = 6;

		public static final int READY_BOTTOM = 9;
		public static final int READY_MIDDLE = 7;
		public static final int READY_TOP = 8;
		public static final int SUBSTATION_PICKUP = 10;

		public static final int TILT_INTAKE_UP = 4;
		public static final int TILT_INTAKE_DOWN = 2;
	}

	public static class SwerveConstants {

		public static final double MAX_LINEAR_SPEED =
			((1276 * 9.42) / 60) / 12 * 0.3048; // 1276 is rpm, 9.42 is wheel circumference (in.), final units are m/s
		public static final double MAX_LINEAR_ACCELERATION = 4; //Test
		public static final double MAX_ROTATIONAL_SPEED =
			MAX_LINEAR_SPEED / (4 / 3); // 4/3 is (about) the radius from the center of the robot to the swerve drive wheels.
		public static final double MAX_ROTATIONAL_ACCELERATION = 4; // Linear Acceleration/radius

		public static final double DRIVE_RATIO = 1 / 5.; // drive rotations per motor rotation
		public static final double ANGLE_RATIO = 1 / 6.75; // angle rotations per motor rotation

		public static final int DBRPORT = 9; //drive back right port
		public static final int DBLPORT = 2; //drive back left port
		public static final int DFLPORT = 3; //drive front left port
		public static final int DFRPORT = 4; //drive front right port
		public static final int ABRPORT = 5; //angle back right port
		public static final int ABLPORT = 6; //angle back left port
		public static final int AFLPORT = 7; //angle front left port
		public static final int AFRPORT = 8; //angle front right port

		public static final double WHEEL_DIAMETER = 0.0762; // in meters

		// These are the translations of the swerve modules from the center of the robot.
		// Specifically, these measurments should land on the line that the swerve module wheel rotates around
		// Units are meters
		public static final Translation2d FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(
			0.3175,
			0.2413
		);
		public static final Translation2d FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(
			0.3175,
			-0.2413
		);
		public static final Translation2d BACK_LEFT_MODULE_TRANSLATION = new Translation2d(
			-0.3175,
			0.2413
		);
		public static final Translation2d BACK_RIGHT_MODULE_TRANSLATION = new Translation2d(
			-0.3175,
			-0.2413
		);
	}

	public static class ArmConstants {

		/** The CAN ID of the arm angle motor */
		public static final int ANGLE_MOTOR_ID = 10;
		/** The CAN ID of the arm extension motor */
		public static final int EXTENSION_MOTOR_ID = 15;

		/** The extension of the arm when it is fully retracted */
		public static final double ZERO_EXTENSION = 0.0;
		/** The extension of the arm when it is picking up from the ground */
		public static final double GROUND_EXTENSION = 0.0;
		/** The extension of the arm when it is placing middle level game pieces */
		public static final double MIDDLE_EXTENSION = 4.78;
		/** The extension of the arm when it is placing high level game pieces */
		public static final double HIGH_EXTENSION = 17.00;
		/** The extension of the arm when it is picking up from the substation */
		public static final double SUBSTATION_EXTENSION = 8.62;

		/** The angle of the arm to remove the latch holding it up when a match starts */
		public static final double START_ANGLE = 0.0;
		/** The angle of the arm when it is traveling or zeroed*/
		public static final double ZERO_ANGLE = -10;
		/** The angle of the arm when it is placing game pieces */
		public static final double PLACE_ANGLE = -10;
		/** The angle of the arm when it is picking up from the substation */
		public static final double SUBSTATION_ANGLE = -52.51;
		/** The angle of the arm when it is picking up from the ground */
		public static final double GROUND_ANGLE = 0.0;
	}

	public static class IntakeConstants {

		/** The CAN ID of the intake output motor */
		public static final int OUTPUT_MOTOR_ID = 13;
		/** The CAN ID of the intake angle motor */
		public static final int ANGLE_MOTOR_ID = 12;

		/** The percent output of the intake while picking up cones */
		public static final double PICKUP_CONE_OUTPUT = .8;
		/** The percent output of the intake while picking up cubes */
		public static final double PICKUP_CUBE_OUTPUT = -.9;
		/** The percent output of the intake while placing cones */
		public static final double PLACE_CONE_OUTPUT = -1;
		/** The percent output of the intake while placing cubes */
		public static final double PLACE_CUBE_OUTPUT = .8;

		/** Intake angle for traveling or zeroing*/
		public static final double ZERO_ANGLE = -27;
		/** Intake angle for picking up game pieces from the ground */
		public static final double GROUND_ANGLE = -27; // TODO: Find this value
		/** Intake angle for placing middle level game pieces  */
		public static final double MIDDLE_ANGLE = -140;
		/** Intake angle for placing top level game pieces */
		public static final double TOP_ANGLE = -70;
		/** Intake angle for picking up game pieces from the substation */
		public static final double SUBSTATION_ANGLE = -140;
	}

	public static class EnumConstants {

		public static enum GamePiece {
			Cube(IntakeConstants.PLACE_CUBE_OUTPUT),
			Cone(IntakeConstants.PLACE_CONE_OUTPUT),
			Nothing(0.0);

			private double intakeOutput;

			private GamePiece(double intakeOutput) {
				this.intakeOutput = intakeOutput;
			}

			public double getIntakeOutput() {
				return intakeOutput;
			}
		}

		public static enum ArmPosition {
			Start(
				ArmConstants.ZERO_EXTENSION,
				ArmConstants.START_ANGLE,
				IntakeConstants.ZERO_ANGLE
			),
			Zero(
				ArmConstants.ZERO_EXTENSION,
				ArmConstants.ZERO_ANGLE,
				IntakeConstants.ZERO_ANGLE
			),
			Bot(
				ArmConstants.GROUND_EXTENSION,
				ArmConstants.PLACE_ANGLE,
				IntakeConstants.GROUND_ANGLE
			),
			Mid(
				ArmConstants.MIDDLE_EXTENSION,
				ArmConstants.PLACE_ANGLE,
				IntakeConstants.MIDDLE_ANGLE
			),
			Top(
				ArmConstants.HIGH_EXTENSION,
				ArmConstants.PLACE_ANGLE,
				IntakeConstants.TOP_ANGLE
			),
			Sub(
				ArmConstants.SUBSTATION_EXTENSION,
				ArmConstants.SUBSTATION_ANGLE,
				IntakeConstants.SUBSTATION_ANGLE
			);

			private double armExtension;
			private double armAngle;
			private double intakeAngle;

			public double getArmExtension() {
				return armExtension;
			}

			public double getArmAngle() {
				return armAngle;
			}

			public double getIntakeAngle() {
				return intakeAngle;
			}

			private ArmPosition(
				double armExtension,
				double armAngle,
				double intakeAngle
			) {
				this.armExtension = armExtension;
				this.armAngle = armAngle;
				this.intakeAngle = intakeAngle;
			}
		}

		public static enum IntakeMode {
			PickupCube(IntakeConstants.PICKUP_CUBE_OUTPUT),
			PickupCone(IntakeConstants.PICKUP_CONE_OUTPUT),
			Place(0.0), // The value for place doesn't matter, since that is decided by which game piece we have
			Off(0.0);

			private double intakeOutput;

			private IntakeMode(double intakeOutput) {
				this.intakeOutput = intakeOutput;
			}

			public double getIntakeOutput() {
				return intakeOutput;
			}
		}

		public static enum AutoDriveMode {
			AprilTagAlign,
			GamePieceAlign,
			RelativePoseAlign,
		}

		public static enum VisionTarget {
			AprilTag(0, 0, 0),
			ReflectiveTape(0, 1, 0),
			GamePiece(1, 0, 7);

			public int limelightId;
			public int pipeline;
			public double setpoint;

			private VisionTarget(
				int limelightId,
				int pipeline,
				double setpoint
			) {
				this.limelightId = limelightId;
				this.pipeline = pipeline;
				this.setpoint = setpoint;
			}
		}

		public static enum TalonType {
			TalonSRX("Talon SRX"),
			TalonFX("Talon FX");

			public String model;

			private TalonType(String model) {
				this.model = model;
			}
		}
	}

	public static class AutoConstants {

		/** The maximum velocity the robot will travel at during auto <p> Units are meters per second*/
		public static final double AUTO_MAX_SPEED = 2;
		/** The maximum acceleration the robot will travel at during auto <p> Units are meters per second*/
		public static final double AUTO_MAX_ACCEL = 2.0;

		public static final List<PathPlannerTrajectory> BlueBotRedTop2PieceTopAuto = PathPlanner.loadPathGroup(
			"BlueBotRedTop2PieceTop",
			AUTO_MAX_SPEED,
			AUTO_MAX_ACCEL
		);

		public static final List<PathPlannerTrajectory> BlueBotRedTop2PieceMidAuto = PathPlanner.loadPathGroup(
			"BlueBotRedTop2PieceMid",
			AUTO_MAX_SPEED,
			AUTO_MAX_ACCEL
		);

		public static final List<PathPlannerTrajectory> BlueBotRedTopPlaceAndDockAuto = PathPlanner.loadPathGroup(
			"BlueBotRedTopPlaceAndDock",
			AUTO_MAX_SPEED,
			AUTO_MAX_ACCEL
		);

		public static final List<PathPlannerTrajectory> BlueTopPlaceAndRunAuto = PathPlanner.loadPathGroup(
			"BlueTopPlaceAndRun",
			AUTO_MAX_SPEED,
			AUTO_MAX_ACCEL
		);

		public static final List<PathPlannerTrajectory> BlueTopRedBot2PieceTopAuto = PathPlanner.loadPathGroup(
			"BlueTopRedBot2PieceTop",
			AUTO_MAX_SPEED,
			AUTO_MAX_ACCEL
		);
		public static final List<PathPlannerTrajectory> BlueTopRedBot2PieceMidAuto = PathPlanner.loadPathGroup(
			"BlueTopRedBot2PieceMid",
			AUTO_MAX_SPEED,
			AUTO_MAX_ACCEL
		);

		public static final List<PathPlannerTrajectory> BlueTopRedBotPlaceAndDockAuto = PathPlanner.loadPathGroup(
			"BlueTopRedBotPlaceAndDock",
			AUTO_MAX_SPEED,
			AUTO_MAX_ACCEL
		);

		public static final List<PathPlannerTrajectory> MidPlaceAndDockAuto = PathPlanner.loadPathGroup(
			"MidPlaceAndDock",
			1,
			1
		);

		public static final List<PathPlannerTrajectory> PlaceAndMoveAuto = PathPlanner.loadPathGroup(
			"PlaceAndMove",
			AUTO_MAX_SPEED,
			AUTO_MAX_ACCEL
		);

		public static final List<PathPlannerTrajectory> RedTopPlaceAndRunAuto = PathPlanner.loadPathGroup(
			"RedTopPlaceAndRun",
			AUTO_MAX_SPEED,
			AUTO_MAX_ACCEL
		);
		// Add Auto Paths here, like the above
	}

	public static class TelemetryConstants {

		public static Mode getMode() {
			return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
		}

		public static enum Mode {
			/** Running on a real robot. */
			REAL,

			/** Running a physics simulator. */
			SIM,

			/** Replaying from a log file.
			 *  TODO: Setup project for REPLAY mode.
			 */
			REPLAY,
		}
	}
}
