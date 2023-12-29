package frc.robot.commands.debugCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.placer.arm.Arm;

public class TiltArmCommand extends CommandBase {

  private Arm arm;
  private double tiltChange;

  public TiltArmCommand(double tiltChange) {
    this.arm = Arm.getInstance();
    this.tiltChange = tiltChange;
  }

  public void initialize() { arm.changeAngle(tiltChange); }

  public boolean isFinished() { return true; }
}
