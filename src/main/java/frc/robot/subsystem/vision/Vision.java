package frc.robot.subsystem.vision;

import frc.robot.component.LimelightVisionComponent;

public class Vision extends LimelightVisionComponent {

	private static Vision instanceVision;

	private Vision() {
		super();
	}

	/**
	 * Gets the instance of the vision. If the instance doesn't exist, it creates it
	 * @return the instance of the swerve drive
	 */
	public static synchronized Vision getInstance() {
		if (instanceVision == null) {
			instanceVision = new Vision();
		}
		return instanceVision;
	}
}
