/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.component.hardware;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.component.GyroComponent;

/**
 * An {@link GyroComponent} wrapper for {@link AHRS}.
 * All getters return radians.
 */
public class AHRSAngleGetterComponent extends AHRS implements GyroComponent {

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
     * Returns the total accumulated yaw angle (Z Axis, in radians)
     * reported by the sensor.
     *<p>
     * NOTE: The angle is continuous, meaning it's range is beyond 2pi radians.
     * This ensures that algorithms that wouldn't want to see a discontinuity 
     * in the gyro output as it sweeps past 0 on the second time around.
     *<p>
     * Note that the returned yaw value will be offset by a user-specified
     * offset value; this user-specified offset value is set by 
     * invoking the zeroYaw() method.
     *<p>
     * @return The current total accumulated yaw angle (Z axis) of the robot 
     * in radians. This heading is based on integration of the returned rate 
     * from the Z-axis (yaw) gyro.
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

    public void reset() {}
}
