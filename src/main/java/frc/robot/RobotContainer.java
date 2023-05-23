package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystem.*;
import frc.robot.subsystem.swerve.SwerveDrive;

public class RobotContainer {

	private final SwerveDrive swerve = SwerveDrive.getInstance();
	private final Arm arm = Arm.getInstance();
	private final Intake intake = Intake.getInstance();

	/* Setting controller Buttons */
	private final DriveController driveStick = DriveController.getInstance(
		JoystickConstants.DRIVER_PORT,
		intake,
		arm,
		swerve
	);
	private final OperatorController controlJoystick = OperatorController.getInstance(
		JoystickConstants.OPERATOR_PORT,
		intake,
		arm
	);

	private final Autonomous autonomous = Autonomous.getInstance(
		swerve,
		arm,
		intake
	);

	public RobotContainer() {}

	public Command getAutonomousCommand() {
		return autonomous.getAutonCommand();
	}

	public void teleopInit() {
		Command auton = autonomous.getAutonCommand();
		if (auton != null) {
			auton.cancel();
		}
	}

	public void disabledInit() {
		swerve.zeroModules();
	}
}
