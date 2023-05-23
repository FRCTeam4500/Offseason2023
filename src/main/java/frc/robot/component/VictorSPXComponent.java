package frc.robot.component;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class VictorSPXComponent extends VictorSPX {

	public VictorSPXComponent(int port) {
		super(port);
	}

	public void setOutput(double output) {
		set(ControlMode.PercentOutput, output);
	}

	public double getOutput() {
		//return getMotorOutputPercent();
		return 0;
	}
}
