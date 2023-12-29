/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.hardware;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;

public class Gyro extends AHRS {

	/**
	 * @see AHRS#AHRS(Port)
	 */
	public Gyro(edu.wpi.first.wpilibj.I2C.Port kmxp) {
		super(kmxp);
	}

	public Gyro(Port kmxp) {
		super(kmxp);
	}

	/**
	 * Gets the absolute angle of the gyro from the angle where power was turned on
	 * @return the absolute angle of the robot in radians. Counter-Clockwise is positive
	 */
	@Override
	public double getAngle() {
		return -Math.toRadians(super.getAngle());
	}

	@Override
	public float getPitch() {
		return (float) Math.toRadians(super.getPitch());
	}

	
	@Override
	public float getRoll() {
		return (float) Math.toRadians(super.getRoll());
	}

	public void reset() {
		super.reset();
	}
}
