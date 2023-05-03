package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.Constants.EnumConstants.IntakeSpeed;
import frc.robot.subsystem.Intake;

public class SetIntakeSpeedCommand extends CommandBase{
    private Intake intake;
    private IntakeSpeed targetSpeed;
    public SetIntakeSpeedCommand(Intake intake, IntakeSpeed speed) {
        this.intake = intake;
        this.targetSpeed = speed;
    }

    public void initialize() {
        switch (targetSpeed) {
            case PickupCube:
                Intake.setGamePiece(GamePiece.Cube);
            case PlaceCone: 
                intake.setSpeed(IntakeConstants.INTAKE_CUBE_SPEED);
                break;
            case PickupUprightCone:
                Intake.setGamePiece(GamePiece.UprightCone);
                intake.setSpeed(IntakeConstants.INTAKE_CONE_SPEED);
                break;
            case PickupTiltedCone:
                Intake.setGamePiece(GamePiece.TiltedCone);
                intake.setSpeed(IntakeConstants.INTAKE_CONE_SPEED);
                break;
            case PlaceCube: 
                intake.setSpeed(IntakeConstants.INTAKE_CONE_SPEED);
                break;
            case Off:
                intake.setSpeed(0);
                break;
        }
    }

    public boolean isFinished() {
        return true;
    }
}
