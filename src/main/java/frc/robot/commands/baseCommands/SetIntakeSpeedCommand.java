package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.Constants.EnumConstants.IntakeMode;
import frc.robot.subsystems.placer.intake.Intake;

public class SetIntakeSpeedCommand extends CommandBase {

  private Intake intake;
  private IntakeMode intakeMode;
  private GamePiece gamePiece;

  public SetIntakeSpeedCommand(IntakeMode mode) { this.intakeMode = mode; }

  @Override
  public void initialize() {
    intake = Intake.getInstance();
    gamePiece = Intake.getGamePiece();
    if (intakeMode == IntakeMode.Place) {
      intake.setOutput(gamePiece.intakeOutput);
    } else {
      intake.setOutput(intakeMode.intakeOutput);
    }
    if (intakeMode.newGamePiece != null) {
      Intake.setGamePiece(intakeMode.newGamePiece);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
