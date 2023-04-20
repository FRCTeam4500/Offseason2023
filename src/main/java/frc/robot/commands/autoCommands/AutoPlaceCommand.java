package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand.Position;
import frc.robot.commands.teleOpCommands.SetConsecutiveIntakeOutputs;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

public class AutoPlaceCommand extends CommandBase{
    private Arm arm;
    private Intake intake;
    private PlacePosition position;
    private boolean isFinished;
    public AutoPlaceCommand(Arm arm, Intake intake, PlacePosition position) {
        this.arm = arm;
        this.intake = intake;
        this.position = position;
        isFinished = false;
    }

    public void initialize() {
        switch (position) {
            case HighCone:
                new SequentialCommandGroup(
                    new SetArmAndIntakeCommand(arm, intake, Position.High), 
                    new WaitCommand(0.5),
                    new SetConsecutiveIntakeOutputs(intake, IntakeConstants.INTAKE_CUBE_SPEED, 0.5, 0),
                    new WaitCommand(0.5),
                    new SetArmAndIntakeCommand(arm, intake, Position.Zero),
                    new InstantCommand(() -> isFinished = true)
                );
                break;
            case HighCube:
                new SequentialCommandGroup(
                    new SetArmAndIntakeCommand(arm, intake, Position.High), 
                    new WaitCommand(0.5),
                    new SetConsecutiveIntakeOutputs(intake, IntakeConstants.INTAKE_CONE_SPEED, 0.5, 0),
                    new WaitCommand(0.5),
                    new SetArmAndIntakeCommand(arm, intake, Position.Zero),
                    new InstantCommand(() -> isFinished = true)
                );
                break;
            case MiddleCone:
                new SequentialCommandGroup(
                    new SetArmAndIntakeCommand(arm, intake, Position.Middle), 
                    new WaitCommand(0.5),
                    new SetConsecutiveIntakeOutputs(intake, IntakeConstants.INTAKE_CUBE_SPEED, 0.5, 0),
                    new WaitCommand(0.5),
                    new SetArmAndIntakeCommand(arm, intake, Position.Zero),
                    new InstantCommand(() -> isFinished = true)
                );
                break;
            case MiddleCube:
                new SequentialCommandGroup(
                    new SetArmAndIntakeCommand(arm, intake, Position.Middle), 
                    new WaitCommand(0.5),
                    new SetConsecutiveIntakeOutputs(intake, IntakeConstants.INTAKE_CONE_SPEED, 0.5, 0),
                    new WaitCommand(0.5),
                    new SetArmAndIntakeCommand(arm, intake, Position.Zero),
                    new InstantCommand(() -> isFinished = true)
                );
                break;
        }
    }

    public void execute() {}

    public enum PlacePosition {
        HighCone,
        HighCube,
        MiddleCone,
        MiddleCube
    }

    public boolean isFinished() {
        return isFinished;
    } 

    public void end(boolean interrupted) {}
}
