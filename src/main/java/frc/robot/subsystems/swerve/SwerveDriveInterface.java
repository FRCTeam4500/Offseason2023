package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveDriveInterface {
  @AutoLog
  public static class DriveInputs {

    public double frontLeftModuleDriveVelocity = 0.0; // m/s
    public double frontLeftModuleAngleRad = 0.0;
    public double frontRightModuleDriveVelocity = 0.0; // m/s
    public double frontRightModuleAngleRad = 0.0;
    public double backLeftModuleDriveVelocity = 0.0; // m/s
    public double backLeftModuleAngleRad = 0.0;
    public double backRightModuleDriveVelocity = 0.0; // m/s
    public double backRightModuleAngleRad = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveInputs inputs) {}
}
