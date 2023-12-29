package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.EnumConstants.VisionTarget;
import frc.robot.commands.baseCommands.RumbleCommand;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class AutoDriveToCommand extends CommandBase {
  private SwerveDrive swerve;
  private Vision vision;
  private VisionTarget target;
  private PIDController pid;
  private double targetArea;
  private double distanceThreshold;

  public AutoDriveToCommand(VisionTarget targetType) {
    swerve = SwerveDrive.getInstance();
    vision = Vision.getInstance();
    pid = new PIDController(1, 0, 0);
    target = targetType;
    distanceThreshold = 0.25;
    addRequirements(swerve, vision);
  }

  @Override
  public void initialize() {
    pid.reset();
    pid.setSetpoint(target.setpoint);
    pid.setTolerance(distanceThreshold);
    vision.setPipeline(target.limelightId, target.pipeline);
    targetArea = vision.getTakenArea(target.limelightId);
    if (!vision.hasValidTargets(target.limelightId)) {
      MessagingSystem.getInstance().addMessage(
          "A " + getName() + "was scheduled, but there were no valid targets!");
      CommandScheduler.getInstance().schedule(new RumbleCommand(0.5));
      CommandScheduler.getInstance().cancel(this);
    }
  }

  @Override
  public void execute() {
    targetArea = vision.getTakenArea(target.limelightId);
    if (vision.hasValidTargets(target.limelightId)) {
      swerve.driveRobotCentric(pid.calculate(targetArea) / 2, 0, 0);
    } else {
      swerve.driveRobotCentric(0, 0, 0);
    }
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.driveRobotCentric(0, 0, 0);
  }
}
