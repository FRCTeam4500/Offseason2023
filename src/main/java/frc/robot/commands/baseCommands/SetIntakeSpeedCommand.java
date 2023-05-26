package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.Constants.EnumConstants.IntakeSpeed;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystem.placer.intake.Intake;

public class SetIntakeSpeedCommand extends CommandBase {

	private Intake intake;
	private IntakeSpeed targetSpeed;
	private double rawTargetSpeed;

	public SetIntakeSpeedCommand(Intake intake, IntakeSpeed speed) {
		this.intake = intake;
		this.targetSpeed = speed;
	}

	@Override
	public void initialize() {
		switch (targetSpeed) {
			case PickupCube:
				Intake.setGamePiece(GamePiece.Cube);
				rawTargetSpeed = IntakeConstants.INTAKE_CUBE_SPEED;
				break;
			case PlaceCone:
				Intake.setGamePiece(GamePiece.Nothing);
				rawTargetSpeed = IntakeConstants.OUTTAKE_CONE_SPEED;
				break;
			case PickupUprightCone:
				Intake.setGamePiece(GamePiece.UprightCone);
				rawTargetSpeed = IntakeConstants.INTAKE_CONE_SPEED;
				break;
			case PickupTiltedCone:
				Intake.setGamePiece(GamePiece.TiltedCone);
				rawTargetSpeed = IntakeConstants.INTAKE_CONE_SPEED;
				break;
			case PlaceCube:
				Intake.setGamePiece(GamePiece.Nothing);
				rawTargetSpeed = IntakeConstants.OUTTAKE_CUBE_SPEED;
				break;
			case Off:
				rawTargetSpeed = 0;
				break;
		}
		intake.setOutput(rawTargetSpeed);
	}

	@Override
	public boolean isFinished() {
		return (
			Math.abs(rawTargetSpeed - intake.getOutput()) <
			IntakeConstants.INTAKE_SPEED_THRESHOLD
		);
	}
}
