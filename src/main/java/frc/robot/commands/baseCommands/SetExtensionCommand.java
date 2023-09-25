package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.subsystems.placer.arm.Arm;

public class SetExtensionCommand extends CommandBase{
    private ArmPosition position;
    private double targetWinchPosition;
    public SetExtensionCommand(ArmPosition position) {
        this.position = position;
    }

    @Override
    public void initialize() {
        switch(position) {
			case Start:
				targetWinchPosition = ArmConstants.ARM_EXTENSION_ZERO;
				break;
			case Zero:
				targetWinchPosition = ArmConstants.ARM_EXTENSION_ZERO;
				break;
			case Bot:
			case Mid:
				targetWinchPosition = ArmConstants.ARM_EXTENTION_MIDDLE;
				break;
			case Top:
				targetWinchPosition = ArmConstants.ARM_EXTENTION_HIGH;
				break;
			case Sub:
				targetWinchPosition = ArmConstants.ARM_EXTENTION_SUBSTATION;
				break;
		}
		Arm.getInstance().setExtension(targetWinchPosition);
    }

    public boolean isFinished() {
        return Math.abs(targetWinchPosition - Arm.getInstance().getExtension()) < 5;
    }
}
