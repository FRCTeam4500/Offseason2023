package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.Constants.EnumConstants.IntakeMode;
import frc.robot.subsystems.placer.intake.Intake;
import frc.robot.Constants.IntakeConstants;

public class SetIntakeSpeedCommand extends CommandBase {

	private Intake intake;
	private IntakeMode intakeMode;
	private GamePiece gamePiece;
	private double rawTargetSpeed;

	public SetIntakeSpeedCommand(IntakeMode mode) {
		this.intakeMode = mode;
	}

	@Override
	public void initialize() {
		intake = Intake.getInstance();
		gamePiece = Intake.getGamePiece();
		switch (intakeMode) {
			case Place:
				switch(gamePiece) {
					case Cone: rawTargetSpeed = IntakeConstants.OUTTAKE_CONE_SPEED; break;
					case Cube: rawTargetSpeed = IntakeConstants.OUTTAKE_CUBE_SPEED; break;
					case Nothing: rawTargetSpeed = 0; break;
				}
				Intake.setGamePiece(GamePiece.Nothing);
				break;
			case PickupCube: 
				rawTargetSpeed = IntakeConstants.INTAKE_CUBE_SPEED; 
				Intake.setGamePiece(GamePiece.Cube); 
				break;
			case PickupCone: 
				rawTargetSpeed = IntakeConstants.INTAKE_CONE_SPEED;
				Intake.setGamePiece(GamePiece.Cone); 
				break;
			case Off: rawTargetSpeed = 0; break;
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
