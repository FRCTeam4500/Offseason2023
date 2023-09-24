package frc.robot.autonomous;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.Constants.EnumConstants.IntakeMode;
import frc.robot.commands.baseCommands.ResetGyroCommand;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.complexCommands.AutoPickupCommand;
import frc.robot.commands.complexCommands.AutoPlaceCommand;
import frc.robot.commands.complexCommands.AutomatedDriveCommand;
import frc.robot.commands.complexCommands.AutomatedDriveCommand.AutoDriveMode;
import frc.robot.subsystems.placer.arm.Arm;
import frc.robot.subsystems.placer.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.commands.complexCommands.ZeroCommand;

import java.util.HashMap;

public class Autonomous {

	private static Autonomous autonomous = null;

	SwerveDrive swerve;
	Arm arm;
	Intake intake;

	/** A hash map containing the commands the robot will use in auto <p> These commands can be accessed by putting the cooresponding string key into the .get() method
	 * <p> Example: {@code autoCommandMap.get("zero");}
	 */
	public static final HashMap<String, Command> autoCommandMap = new HashMap<>();

	/**Both PID constants need to be tested */
	private final SwerveAutoBuilder autoBuilder;
	private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

	private Autonomous() {
		swerve = SwerveDrive.getInstance();
		arm = Arm.getInstance();
		intake = Intake.getInstance();
		this.autoBuilder =
			new SwerveAutoBuilder(
				swerve::getRobotPose,
				swerve::resetPose,
				new PIDConstants(5, 0, 0),
				new PIDConstants(4, 0, 0),
				swerve::driveModules,
				autoCommandMap,
				swerve
			);

		configureAuto();
	}

	private void configureAuto() {
		autoCommandMap.put(
			"start",
			new SetArmAndIntakeCommand(ArmPosition.Start)
		);

		autoCommandMap.put("zero", new ZeroCommand());

		autoCommandMap.put(
			"placeCubeTop",
			new AutoPlaceCommand(ArmPosition.Top)
		);

		autoCommandMap.put(
			"placeConeTop",
			new AutoPlaceCommand(ArmPosition.Top)
		);

		autoCommandMap.put(
			"placeCubeMid",
			new AutoPlaceCommand(ArmPosition.Mid)
		);

		autoCommandMap.put(
			"placeConeMid",
			new AutoPlaceCommand(ArmPosition.Mid)
		);

		autoCommandMap.put("resetGyro", new ResetGyroCommand());

		autoCommandMap.put("reverseGyro", new ResetGyroCommand(180));

		autoCommandMap.put(
			"pickupCone",
			new AutoPickupCommand(IntakeMode.PickupCone)
		);

		autoCommandMap.put(
			"pickupCube",
			new AutoPickupCommand(IntakeMode.PickupCube)
		);

		autonChooser.setDefaultOption(
			"Auto Driving Test",
			new AutomatedDriveCommand(
				swerve,
				AutoDriveMode.kRelative,
				0.1,
				0.1,
				1,
				new Pose2d(2, 0, new Rotation2d()),
				new Pose2d(0, 0, new Rotation2d())
			)
		);
		autonChooser.addOption(
			"Blue Bottom: 2 Piece Top",
			autoBuilder.fullAuto(AutoConstants.BlueBotRedTop2PieceTopAuto)
		);
		autonChooser.addOption(
			"Red Top: 2 Piece Top",
			autoBuilder.fullAuto(AutoConstants.BlueBotRedTop2PieceTopAuto)
		);
		autonChooser.addOption(
			"Blue Top: 2 Piece Top",
			autoBuilder.fullAuto(AutoConstants.BlueTopRedBot2PieceTopAuto)
		);
		autonChooser.addOption(
			"Red Bottom: 2 Piece Top",
			autoBuilder.fullAuto(AutoConstants.BlueTopRedBot2PieceTopAuto)
		);
		autonChooser.addOption(
			"Blue Bottom: 2 Piece Mid",
			autoBuilder.fullAuto(AutoConstants.BlueBotRedTop2PieceMidAuto)
		);
		autonChooser.addOption(
			"Red Top: 2 Piece Mid",
			autoBuilder.fullAuto(AutoConstants.BlueBotRedTop2PieceMidAuto)
		);
		autonChooser.addOption(
			"Blue Top: 2 Piece Mid",
			autoBuilder.fullAuto(AutoConstants.BlueTopRedBot2PieceMidAuto)
		);
		autonChooser.addOption(
			"Red Bottom: 2 Piece Mid",
			autoBuilder.fullAuto(AutoConstants.BlueTopRedBot2PieceMidAuto)
		);
		autonChooser.addOption(
			"Both Middle: Place, Mobility, and Dock",
			autoBuilder.fullAuto(AutoConstants.MidPlaceAndDockAuto)
		);
		autonChooser.addOption(
			"Both Side: Place 1 and Mobility",
			autoBuilder.fullAuto(AutoConstants.PlaceAndMoveAuto)
		);
		autonChooser.addOption(
			"Red Top: Place and Face Substation",
			autoBuilder.fullAuto(AutoConstants.RedTopPlaceAndRunAuto)
		);
		autonChooser.addOption(
			"Blue Top: Place and Face Substation",
			autoBuilder.fullAuto(AutoConstants.BlueTopPlaceAndRunAuto)
		);

		Shuffleboard.getTab("Auto").add("Auto Routes", autonChooser);
	}

	/**
	 * Returns the instance of the autonomous class. If the instance is null, it will create a new instance.
	 * @return
	 */
	public static synchronized Autonomous getInstance() {
		if (autonomous == null) {
			autonomous = new Autonomous();
		}
		return autonomous;
	}

	/**
	 * Returns the selected autonomous command.
	 * @return
	 */
	public Command getAutonCommand() {
		return autonChooser.getSelected();
	}
}
