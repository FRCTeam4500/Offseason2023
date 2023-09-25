package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.subsystems.placer.arm.Arm;
import frc.robot.subsystems.placer.intake.Intake;

public class SetTiltCommand extends CommandBase{
    private ArmPosition position;
    private double targetArmAngle;
    private double targetIntakeAngle;
    public SetTiltCommand(ArmPosition position) {
        this.position = position;
    }

    @Override
    public void initialize() {
        switch(position) {
			case Start:
				targetArmAngle = 0;
				targetIntakeAngle = IntakeConstants.INTAKE_ZERO_TILT;
				break;
			case Zero:
				targetArmAngle = ArmConstants.ARM_TILT_PLACE;
				targetIntakeAngle = IntakeConstants.INTAKE_ZERO_TILT;
				break;
			case Bot:
			case Mid:

				targetArmAngle = ArmConstants.ARM_TILT_PLACE;
				targetIntakeAngle = IntakeConstants.INTAKE_PLACE_ANGLE;
				break;
			case Top:
				targetArmAngle = ArmConstants.ARM_TILT_PLACE;
				targetIntakeAngle = IntakeConstants.INTAKE_HIGH_ANGLE;
				break;
			case Sub:
				targetArmAngle = ArmConstants.ARM_TILT_SUBSTATION;
				targetIntakeAngle = IntakeConstants.INTAKE_PLACE_ANGLE;
				break;
		}
		Arm.getInstance().setAngle(targetArmAngle);
		Intake.getInstance().setAngle(targetIntakeAngle);
    }

    public boolean isFinished() {
        return Math.abs(targetArmAngle - Arm.getInstance().getAngle()) < 5;
    }
}
