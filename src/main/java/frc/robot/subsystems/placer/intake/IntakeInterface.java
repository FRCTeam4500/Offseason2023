package frc.robot.subsystems.placer.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeInterface {
	@AutoLog
	public class IntakeInputs {

		public double intakeAngleMotorRot = 0.0;
		public double intakeOutput = 0.0;
	}

	public default void updateInputs(IntakeInputs inputs) {}
}
