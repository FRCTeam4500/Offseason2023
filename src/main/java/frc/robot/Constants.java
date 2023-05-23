package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
		public static final int UPRIGHT_CONE_INTAKE = 6;
		public static final int TILTED_CONE_INTAKE = 5;

		public static final int READY_BOTTOM = 9;
		public static final int READY_MIDDLE = 7;
		public static final int READY_TOP = 8;
		public static final int SUBSTATION_PICKUP = 10;
	}

	public static class SwerveConstants {

		public static final double MAX_LINEAR_SPEED =
			((1276 * 9.42) / 60) / 12 * 0.3048; // 1276 is rpm, 9.42 is wheel circumference (in.), final units are ft/s
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
		// distances from the gyro to the swerve module wheel.
		public static final double DRIVE_X_RIGHT_TRANSLATION = -0.2413;
		public static final double DRIVE_X_LEFT_TRANSLATION = 0.2413;
		public static final double DRIVE_Y_FRONT_TRANSLATION = 0.3175;
		public static final double DRIVE_Y_BACK_TRANSLATION = -0.3175;
	}

	public static class ArmConstants { // Arm Contstants

		/** The CAN ID of the arm tilt motor */
		public static final int TILT_MOTOR_ID = 10;
		/** The motor type of the tilt motor */
		public static final MotorType TILT_MOTOR_TYPE = MotorType.kBrushless;
		/** The CAN ID of the arm winch motor */
		public static final int WINCH_MOTOR_ID = 15;

		/** The angle the arm must be at to pickup game pieces from the high substation <p> Units are whatever shuffleboard says*/
		public static final double ARM_HIGH_SUBSTATION_ANGLE = -3;
		/** The angle the arm must be at to place at the middle level <p> Units are whatever shuffleboard says*/
		public static final double ARM_PLACE_ANGLE = -6;
		/** The angle the arm must be at to launch a cone onto the top node <p> Units are whatever shuffleboard says */
		public static final double ARM_LAUNCH_ANGLE = 1;
		/** The angle the arm must be at to pickup game pieces from the ground <p> Units are whatever shuffleboard says */
		public static final double ARM_GROUND_ANGLE = -43;
		/** The angle the arm will go to while traveling <p> Units are whatever shuffleboard says */
		public static final double ARM_ZERO_ANGLE = -10;

		public static final double ARM_PLACE_TILTED_CONE_ANGLE = 0;

		/** The extension the arm must have to place a game piece on the top node <p> Also used for intaking game pieces off the high substation <p> Units are raw sensor units */
		public static final double ARM_PLACE_TOP = 10900.0;
		/** The extension the arm must have to place a game piece on the middle node <p> Also used to place game pieces on the ground <p> Units are raw sensor units */
		public static final double ARM_PLACE_MID = 3229;
		/** The extension the arm must have to pickup a game piece from the ground <p> Units are raw sensor units*/
		public static final double ARM_PICKUP = 4324;
		/** The extension the arm will have while traveling <p> Units are raw sensor units */
		public static final double ARM_RETRACT = 0;

		public static final double ARM_PLACE_TILTED_CONE_MID = 5000;

		public static final double ARM_WINCH_THRESHOLD = 250;
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

		/** The angle the intake must be at to pickup game pieces from the ground <p> Units are whatever shuffleboard says */
		public static final double INTAKE_BOT_ANGLE = -7;
		/** The angle the intake must be at to pickup games pieces from the high substation <p> Units are whatever shuffleboard says */
		public static final double INTAKE_HIGH_SUBSTATION_ANGLE = -25;
		/** The angle the intake must be at to place a top cone (as in we picked up an upright cone) on a node <p> Units are whatever shuffleboard says */
		public static final double INTAKE_TOP_CONE_PLACE_ANGLE = -18.4;
		/** The angle the intake must be at to place a bottom cone (as in we picked up a sideways cone) on a node <p><strong> This value needs to be updated</strong><p> Units are whatever shuffleboard says */
		public static final double INTAKE_TILTED_CONE_ANGLE = -23.5;
		/** The angle the intake will go to while traveling <p> Units are whatever shuffleboard says */
		public static final double INTAKE_ZERO_ANGLE = -5.5;
		/** The angle the intake must be at to launch a cone onto the top node <p> Units are whatever shuffleboard says */
		public static final double INTAKE_LAUNCHING_ANGLE = -15.5;
	}

	public static class EnumConstants {

		public static enum GamePiece {
			Cube,
			TiltedCone,
			UprightCone,
			Nothing,
		}

		public static enum PlacerState {
			HighUprightCone,
			HighCube,
			MidTiltedCone,
			MidUprightCone,
			MidCube,
			GroundPickup,
			SubstationPickup,
			Zero,
			Start,
		}

		public static enum IntakeSpeed {
			PlaceCone,
			PlaceCube,
			PickupUprightCone,
			PickupTiltedCone,
			PickupCube,
			Off,
		}
	}

	public static class AutoConstants { //Everything in this class must be in meters

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
			return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY; // Just change in init if you really want replay
		}

		public static enum Mode {
			/** Running on a real robot. */
			REAL,

			/** Running a physics simulator. */
			SIM,

			/** Replaying from a log file. */
			REPLAY,
		}
	}
}
