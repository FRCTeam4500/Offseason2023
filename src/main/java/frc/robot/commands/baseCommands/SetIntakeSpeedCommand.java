package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.complexCommands.PlaceCommand.GamePiece;
import frc.robot.subsystem.Intake;

public class SetIntakeSpeedCommand extends CommandBase{
    private Intake intake;
    private Output targetOutput;
    public SetIntakeSpeedCommand(Intake intake, Output output) {
        this.intake = intake;
        this.targetOutput = output;
    }

    public void initialize() {
        switch (targetOutput) {
            case PickupCube:
                Intake.setGamePiece(GamePiece.Cube);
            case PlaceCone: 
                intake.setSpeed(IntakeConstants.INTAKE_CUBE_SPEED);
                break;
            case PickupCone:
                Intake.setGamePiece(GamePiece.Cone);
            case PlaceCube: 
                intake.setSpeed(IntakeConstants.INTAKE_CONE_SPEED);
                break;
            case Zero:
                intake.setSpeed(0);
                break;
        }
    }

    public boolean isFinished() {
        return true;
    }

    public enum Output {
        PlaceCone,
        PlaceCube,
        PickupCone,
        PickupCube,
        Zero
    }
}
