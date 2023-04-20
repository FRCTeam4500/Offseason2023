package frc.robot.commands.baseCommands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.Intake;

public class SetIntakeSpeedCommand extends CommandBase{
    private Intake intake;
    private double targetOutput;
    private double differenceToTargetOutput;
    public SetIntakeSpeedCommand(Intake intake, double output) {
        this.intake = intake;
        targetOutput = output;
    }

    public void initialize() {
        intake.setSpeed(targetOutput);
    }

    public void execute() {
        differenceToTargetOutput = targetOutput - intake.getSpeed();
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted){}

    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target Output", () ->  targetOutput, null);
        builder.addDoubleProperty("Difference to target output", () -> differenceToTargetOutput, null);
    }
}
