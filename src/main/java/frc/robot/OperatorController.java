package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.Constants.EnumConstants.IntakeMode;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.commands.complexCommands.PlaceCommand;
import frc.robot.commands.complexCommands.ZeroCommand;
import frc.robot.commands.debugCommands.TiltIntakeCommand;
import frc.robot.subsystem.placer.arm.Arm;
import frc.robot.subsystem.placer.intake.Intake;
import frc.robot.subsystem.vision.Vision;

public class OperatorController extends CommandJoystick {

	Intake intake;
	Arm arm;

	private static OperatorController instanceOperatorController = null;

	private final Trigger cubeButton =
		this.button(JoystickConstants.CUBE_INTAKE);
	private final Trigger placeButton = this.button(JoystickConstants.PLACE);
	private final Trigger readySubstationButton =
		this.button(JoystickConstants.SUBSTATION_PICKUP);
	private final Trigger readyTopButton =
		this.button(JoystickConstants.READY_TOP);
	private final Trigger readyMidButton =
		this.button(JoystickConstants.READY_MIDDLE);
	private final Trigger readyBotButton =
		this.button(JoystickConstants.READY_BOTTOM);
	private final Trigger coneButton =
		this.button(JoystickConstants.CONE_INTAKE);
	private final Trigger tiltUpButton = this.button(4);
	private final Trigger tiltDownButton = this.button(2);

	private OperatorController() {
		super(JoystickConstants.OPERATOR_PORT);
		intake = Intake.getInstance();
		arm = Arm.getInstance();

		setControllerButtons();
	}

	/**
	 * Creates a new instance of the OperatorController. If the instance is null, it will create a new one.
	 * @return An instance of the OperatorController
	 */
	public static synchronized OperatorController getInstance() {
		if (instanceOperatorController == null) {
			instanceOperatorController =
				new OperatorController();
		}
		return instanceOperatorController;
	}

	public void setControllerButtons() {
		tiltUpButton.toggleOnTrue(new TiltIntakeCommand(intake, 1));
		tiltDownButton.toggleOnTrue(new TiltIntakeCommand(intake, -1));

		cubeButton.toggleOnTrue(new SetIntakeSpeedCommand(IntakeMode.PickupCube));
		cubeButton.toggleOnFalse(new ZeroCommand());

		coneButton.toggleOnTrue(new SetIntakeSpeedCommand(IntakeMode.PickupCone));
		coneButton.toggleOnFalse(new ZeroCommand());

		readyBotButton.toggleOnTrue(new SetArmAndIntakeCommand(ArmPosition.Bot));

		readyMidButton.toggleOnTrue(new SetArmAndIntakeCommand(ArmPosition.Mid));

		readyTopButton.toggleOnTrue(new SetArmAndIntakeCommand(ArmPosition.Top));

		readySubstationButton.toggleOnTrue(new SetArmAndIntakeCommand(ArmPosition.Sub));

		placeButton.toggleOnTrue(new PlaceCommand());

		placeButton.toggleOnFalse(new ZeroCommand());

		Shuffleboard.getTab("Arm and Intake").add("Intake", intake);
		Shuffleboard.getTab("Arm and Intake").add("Arm", arm);
		Shuffleboard.getTab("Vision").add("Vision", Vision.getInstance());
		Shuffleboard
			.getTab("Arm and Intake")
			.addString(
				"Current Game Piece",
				() -> Intake.getGamePiece().get().name()
			);
	}
}
