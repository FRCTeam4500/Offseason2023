package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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

		/** The CAN ID of the arm tilt motor */
		public static final int TILT_MOTOR_ID = 10;
		/** The motor type of the tilt motor */
		public static final MotorType TILT_MOTOR_TYPE = MotorType.kBrushless;
		/** The CAN ID of the arm winch motor */
		public static final int WINCH_MOTOR_ID = 15;


		public static final double ARM_EXTENSION_ZERO = 0.0;
		/** Arm val new hehe	 */
		public static final double ARM_EXTENTION_MIDDLE = 4.78;
		/** Arm ext High */
		public static final double ARM_EXTENTION_HIGH = 17.00;
		/** Arm ext Substation */
		public static final double ARM_EXTENTION_SUBSTATION = 8.62;
		/** Arm tilt Substation */
		public static final double ARM_TILT_SUBSTATION = -52.51;
		/** Arm tilt Placing */
		public static final double ARM_TILT_PLACE = -10;

	}

	public static class IntakeConstants {

		/** The speed of the intake while it is intaking cones and placing cubes <p> Units are percentage of full power */
		public static final double INTAKE_CONE_SPEED = .8;
		/** The speed of the intake while it is intaking cubes and placing cones <p> Units are percentage of full power */
		public static final double INTAKE_CUBE_SPEED = -.9;

		public static final double OUTTAKE_CONE_SPEED = -1;

		public static final double OUTTAKE_CUBE_SPEED = .8;

		public static final double INTAKE_SPEED_THRESHOLD = 0.05;

		/** The CAN ID of the intake run motor */
		public static final int INTAKE_MOTOR_ID = 13;
		/** The motor type of the intake run motor */
		public static final MotorType INTAKE_MOTOR_TYPE = MotorType.kBrushless;
		/** The CAN ID of the intake angle motor */
		public static final int INTAKE_ANGLE_MOTOR_ID = 12;
		/** The motor type of the intake angle motor */
		public static final MotorType ANGLE_MOTOR_TYPE = MotorType.kBrushless;

		/* NEW VALUES WOW */
		
		/** NEW Intake angle hehe max val */
		public static final double INTAKE_PLACE_ANGLE = -140;
		/** NEW Intake angle hehe high only wow */
		public static final double INTAKE_HIGH_ANGLE = -70;

		public static final double INTAKE_ZERO_TILT = -27;
	}

	public static class EnumConstants {

		public static enum GamePiece {
			Cube,
			Cone,
			Nothing,
		}

		public static enum ArmPosition {
			Start,
			Zero,
			Bot,
			Mid,
			Top,
			Sub
		}

		public static enum IntakeMode {
			PickupCube,
			PickupCone,
			Place,
			Off
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
