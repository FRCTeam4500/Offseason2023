package frc.robot.commands.teleOpCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.subsystem.Intake;

public class SetConsecutiveIntakeOutputs extends CommandBase{
    private Intake intake;
    private double firstOutput;
    private double secondOutput;
    private double wait;
    private boolean isFinished;

    public SetConsecutiveIntakeOutputs(Intake intake, double firstOutput, double wait, double secondOutput) {
        this.intake = intake;
        this.firstOutput = firstOutput;
        this.secondOutput = secondOutput;
        this.wait = wait;
        isFinished = false;
    }

    public void initialize() {
        new SequentialCommandGroup(
            new SetIntakeSpeedCommand(intake, firstOutput),
            new WaitCommand(wait),
            new SetIntakeSpeedCommand(intake, secondOutput),
            new InstantCommand(() -> isFinished = true)
        );    
    }

    public void execute(){}

    public boolean isFinished() {
        return isFinished;
    }

    public void end(boolean interrupted) {}
}
