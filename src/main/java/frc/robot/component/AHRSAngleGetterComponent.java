/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.component;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * An {@link GyroComponent} wrapper for {@link AHRS}.
 * All getters return radians.
 */
public class AHRSAngleGetterComponent extends AHRS {

	/**
	 * @see AHRS#AHRS(Port)
	 */
	public AHRSAngleGetterComponent(edu.wpi.first.wpilibj.I2C.Port kmxp) {
		super(kmxp);
	}

	public AHRSAngleGetterComponent(Port kmxp) {
		super(kmxp);
	}

	/**
	 * Gets the absolute angle of the gyro from the angle where power was turned on
	 * @return the absolute angle of the robot in radians. Counter-Clockwise is positive
	 */
	public double getAngle() {
		return -Math.toRadians(super.getAngle());
	}

	/**
	 * Returns the pitch of the robot in radians
	 * @return the pitch of the robot in radians
	 */
	@Override
	public float getPitch() {
		return (float) Math.toRadians(super.getPitch());
	}

	/**
	 * Returns the roll of the robot in radians
	 * @return the roll of the robot in radians
	 */
	@Override
	public float getRoll() {
		return (float) Math.toRadians(super.getRoll());
	}

	public void reset() {}
}
