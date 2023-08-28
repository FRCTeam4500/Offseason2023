package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.Constants.EnumConstants.PlacerState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystem.placer.arm.Arm;
import frc.robot.subsystem.placer.intake.Intake;

/**
 * A command that sets the Arm winch state, the Arm angle, and the Intake angle
 */
public class SetArmAndIntakeCommand extends CommandBase {

	private Arm arm;
	private Intake intake;
	private GamePiece gamePiece;
	private ArmPosition position;

	private double targetWinchPosition;
	private double targetArmAngle;
	private double targetIntakeAngle;

	/**
	 * The constructor of SetArmAndIntakeCommand
	 * @param arm the arm subsystem
	 * @param intake the intake subsystem
	 * @param state the desired state
	 */
	public SetArmAndIntakeCommand(ArmPosition position) {
		this.position = position;
	}

	public void initialize() {
		arm = Arm.getInstance();
		intake = Intake.getInstance();
		gamePiece = Intake.getGamePiece().get();

		switch(position) {
			case Zero:
				targetWinchPosition = ArmConstants.ARM_RETRACT;
				targetArmAngle = ArmConstants.ARM_ZERO_ANGLE;
				targetIntakeAngle = IntakeConstants.INTAKE_ZERO_ANGLE;
				break;
			case Bot:
				targetWinchPosition = ArmConstants.ARM_PICKUP;
				targetArmAngle = ArmConstants.ARM_GROUND_ANGLE;
				targetIntakeAngle = IntakeConstants.INTAKE_BOT_ANGLE;
				break;
			case Mid:
				targetWinchPosition = ArmConstants.ARM_PLACE_MID;
				targetArmAngle = ArmConstants.ARM_PLACE_ANGLE;
				targetIntakeAngle = IntakeConstants.INTAKE_TOP_CONE_PLACE_ANGLE;
				break;
			case Top:
				targetWinchPosition = ArmConstants.ARM_PLACE_TOP;
				targetArmAngle = ArmConstants.ARM_LAUNCH_ANGLE;
				targetIntakeAngle = IntakeConstants.INTAKE_LAUNCHING_ANGLE;
				break;
			case Sub:
				targetWinchPosition = ArmConstants.ARM_PLACE_TOP;
				targetArmAngle = ArmConstants.ARM_HIGH_SUBSTATION_ANGLE;
				targetIntakeAngle = IntakeConstants.INTAKE_HIGH_SUBSTATION_ANGLE;
				break;
		}
		arm.setExtension(targetWinchPosition);
		arm.setAngle(targetArmAngle);
		intake.setAngle(targetIntakeAngle);
	}

	public boolean isFinished() {
		return true;
	}
}
