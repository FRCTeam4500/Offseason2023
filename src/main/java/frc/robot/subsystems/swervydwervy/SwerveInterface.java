package frc.robot.subsystems.swervydwervy;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveInterface {
	@AutoLog
	public static class DriveInputs {

		public double frontLeftModuleDriveVelocity = 0.0; // m/s
		public double frontRightModuleDriveVelocity = 0.0; // m/s
		public double backLeftModuleDriveVelocity = 0.0; // m/s
		public double backRightModuleDriveVelocity = 0.0; // m/s

		public double frontLeftAngleRadians = 0.0; // rads
		public double frontRightAngleRadians = 0.0; // rads
		public double backLeftAngleRadians = 0.0; // rads
		public double backRightAngleRadians = 0.0; // rads
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(DriveInputs inputs) {}
}
