package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.subsystems.placer.arm.Arm;
import frc.robot.subsystems.placer.intake.Intake;

public class SetArmAndIntakeCommand extends CommandBase {
  private Arm arm;
  private Intake intake;
  private ArmPosition position;

  public SetArmAndIntakeCommand(ArmPosition position) {
    this.position = position;
  }

  public void initialize() {
    arm = Arm.getInstance();
    intake = Intake.getInstance();

    arm.setExtension(position.armExtension);
    arm.setAngle(position.armAngle);
    intake.setAngle(position.intakeAngle);
  }

  public boolean isFinished() { return true; }
}
