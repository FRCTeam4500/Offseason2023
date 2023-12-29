package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionInterface {
  @AutoLog
  public class VisionInputs {

    public double horizontalAngleOffset = 0.0;
    public Pose2d robotPose = new Pose2d();
  }

  public default void updateInputs(VisionInputs inputs) {}
}
