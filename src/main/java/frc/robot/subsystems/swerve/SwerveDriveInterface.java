package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveDriveInterface {
	@AutoLog
	public static class DriveInputs {

		// public double frontLeftModuleDriveMeters = 0.0;
		public double frontLeftModuleDriveVelocity = 0.0; // m/s
		public double frontLeftModuleAngleRad = 0.0;
		// public double frontLeftModuleAngleVelocity = 0.0; // rad/s

		// public double frontRightModuleDriveMeters = 0.0;
		public double frontRightModuleDriveVelocity = 0.0; // m/s
		public double frontRightModuleAngleRad = 0.0;
		// public double frontRightModuleAngleVelocity = 0.0; // rad/s

		// public double backLeftModuleDriveMeters = 0.0;
		public double backLeftModuleDriveVelocity = 0.0; // m/s
		public double backLeftModuleAngleRad = 0.0;
		// public double backLeftModuleAngleVelocity = 0.0; // rad/s

		// public double backRightModuleDriveMeters = 0.0;
		public double backRightModuleDriveVelocity = 0.0; // m/s
		public double backRightModuleAngleRad = 0.0;
		// public double backRightModuleAngleVelocity = 0.0; // rad/s

		// public double gyroYawRad = 0.0;
		// public double gyroRollRad = 0.0;
		// public double gyroPitchRad = 0.0;
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(DriveInputs inputs) {}
}
