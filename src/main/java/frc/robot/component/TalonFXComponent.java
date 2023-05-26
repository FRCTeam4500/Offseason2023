/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.component;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * Add your docs here.
 */
public class TalonFXComponent extends TalonFX implements GenericMotorInterface {

	public static final int TICKS_PER_REVOLUTION = 2048;
	public static final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;
	public static final double TICKS_PER_RADIAN =
		TICKS_PER_REVOLUTION / Math.PI / 2.0;

	public TalonFXComponent(int deviceNumber) {
		super(deviceNumber);
	}

	public double getMotorRotations() {
		return getSelectedSensorPosition() / TICKS_PER_REVOLUTION;
	}

	public double getAngle() {
		return getSelectedSensorPosition() / TICKS_PER_RADIAN;
	}

	public void setAngle(double angle) {
		set(TalonFXControlMode.MotionMagic, angle * TICKS_PER_RADIAN);
	}

	public void setOutput(double speed) {
		set(TalonFXControlMode.PercentOutput, speed);
	}

	public double getOutput() {
		return getMotorOutputPercent();
	}

	public double getAngularVelocity() {
		return getSelectedSensorVelocity() / TICKS_PER_RADIAN * 10;
	}

	public void setAngularVelocity(double velocity) {
		set(TalonFXControlMode.Velocity, velocity * TICKS_PER_RADIAN / 10.0);
	}

	public void playFrequency(double frequency) {
		set(TalonFXControlMode.MusicTone, frequency);
	}
}
