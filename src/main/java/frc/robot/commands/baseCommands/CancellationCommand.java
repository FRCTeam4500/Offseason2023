package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CancellationCommand extends CommandBase{
    public CancellationCommand(SubsystemBase[] subsystems) {
        addRequirements(subsystems);
    }

    public boolean isFinished() {
        return true;
    }
}


