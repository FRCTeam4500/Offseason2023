package frc.robot.subsystem.placer.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmInterface {
	@AutoLog
	public class ArmInputs {

		public double tiltMotorRot = 0.0;
		public double extentionMotorRot = 0.0;
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(ArmInputs inputs) {}
}
