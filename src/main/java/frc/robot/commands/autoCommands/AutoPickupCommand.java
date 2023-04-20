package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand.Position;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

public class AutoPickupCommand extends CommandBase{
    private Arm arm;
    private Intake intake;
    private GamePiece piece;
    private boolean isFinished;
    public AutoPickupCommand(Arm arm, Intake intake, GamePiece piece) {
        this.arm = arm;
        this.intake = intake;
        this.piece = piece;
        isFinished = false;
    }
    
    public void initialize() {
        switch(piece) {
            case Cone:
                new SequentialCommandGroup(
                    new SetArmAndIntakeCommand(arm, intake, Position.Low),
                    new SetIntakeSpeedCommand(intake, IntakeConstants.INTAKE_CONE_SPEED),
                    new InstantCommand(() -> isFinished = true)    
                );
                break;
            case Cube:
                new SequentialCommandGroup(
                    new SetArmAndIntakeCommand(arm, intake, Position.Low),
                    new SetIntakeSpeedCommand(intake, IntakeConstants.INTAKE_CUBE_SPEED),
                    new InstantCommand(() -> isFinished = true)    
                );
                break;
        }
    }

    public void execute() {}

    public boolean isFinished() {
        return isFinished;
    }

    public void end(boolean interrupted) {}


    public enum GamePiece {
        Cone,
        Cube
    }
}
