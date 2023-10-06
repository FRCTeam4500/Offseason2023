package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.Constants.EnumConstants.IntakeMode;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.commands.complexCommands.PlaceCommand;
import frc.robot.commands.complexCommands.TeleopZeroCommand;
import frc.robot.commands.debugCommands.TiltIntakeCommand;
import frc.robot.subsystems.placer.arm.Arm;
import frc.robot.subsystems.placer.intake.Intake;

public class OperatorController extends CommandJoystick {

	private static OperatorController instanceOperatorController = null;

	private final Trigger cubeButton =
		this.button(JoystickConstants.CUBE_INTAKE);
	private final Trigger coneButton =
		this.button(JoystickConstants.CONE_INTAKE);
	private final Trigger placeButton = this.button(JoystickConstants.PLACE);
	private final Trigger readySubstationButton =
		this.button(JoystickConstants.SUBSTATION_PICKUP);
	private final Trigger readyTopButton =
		this.button(JoystickConstants.READY_TOP);
	private final Trigger readyMidButton =
		this.button(JoystickConstants.READY_MIDDLE);
	private final Trigger readyBotButton =
		this.button(JoystickConstants.READY_BOTTOM);
	private final Trigger tiltUpButton =
		this.button(JoystickConstants.TILT_INTAKE_UP);
	private final Trigger tiltDownButton =
		this.button(JoystickConstants.TILT_INTAKE_DOWN);

	private OperatorController() {
		super(JoystickConstants.OPERATOR_PORT);
		setButtons();
		Intake.getInstance();
		Arm.getInstance();
	}

	public static synchronized OperatorController getInstance() {
		if (instanceOperatorController == null) {
			instanceOperatorController = new OperatorController();
		}
		return instanceOperatorController;
	}

	public void setButtons() {
		tiltUpButton.toggleOnTrue(new TiltIntakeCommand(1));
		tiltDownButton.toggleOnTrue(new TiltIntakeCommand(-1));

		cubeButton.toggleOnTrue(
			new SetIntakeSpeedCommand(IntakeMode.PickupCube)
				.alongWith(
					new InstantCommand(() -> {
						if (Intake.getInstance().getIsSubstation()) {
							Intake.getInstance().changeAngle(-10);
						}
					})
				)
		);
		cubeButton.toggleOnFalse(new TeleopZeroCommand());

		coneButton.toggleOnTrue(
			new SetIntakeSpeedCommand(IntakeMode.PickupCone)
		);
		coneButton.toggleOnFalse(new TeleopZeroCommand());

		readyBotButton.toggleOnTrue(
			new SetArmAndIntakeCommand(ArmPosition.Bot)
		);
		readyMidButton.toggleOnTrue(
			new SetArmAndIntakeCommand(ArmPosition.Mid)
		);
		readyTopButton.toggleOnTrue(
			new SetArmAndIntakeCommand(ArmPosition.Top)
		);
		readySubstationButton.toggleOnTrue(
			new SetArmAndIntakeCommand(ArmPosition.Sub)
		);

		placeButton.toggleOnTrue(
			new PlaceCommand()
			.andThen(new WaitCommand(0.5))
			.andThen(new TeleopZeroCommand()
		));
	}
}
