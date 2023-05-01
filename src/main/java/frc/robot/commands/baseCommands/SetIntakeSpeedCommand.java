package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
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
            case PlaceCone: case PickupCube:
                intake.setSpeed(IntakeConstants.INTAKE_CUBE_SPEED);
                break;
            case PlaceCube: case PickupCone:
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
