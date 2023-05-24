// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.utility.LogSubsystemInputsTask;
import java.util.Timer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

	private RobotContainer robotContainer;
	private Command autonomousCommand;
	private Timer timer;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		Logger logger = Logger.getInstance();
		timer = new Timer();

		// Record metadata
		logger.recordMetadata("ProjectName", BuildInfo.MAVEN_NAME);
		logger.recordMetadata("BuildDate", BuildInfo.BUILD_DATE);
		logger.recordMetadata("GitSHA", BuildInfo.GIT_SHA);
		logger.recordMetadata("GitDate", BuildInfo.GIT_DATE);
		logger.recordMetadata("GitBranch", BuildInfo.GIT_BRANCH);
		switch (BuildInfo.DIRTY) {
			case 0:
				logger.recordMetadata("GitDirty", "All changes committed");
				break;
			case 1:
				logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;
			default:
				logger.recordMetadata("GitDirty", "Unknown");
				break;
		}
		switch (TelemetryConstants.getMode()) {
			case REAL:
				logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
				logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
				LoggedPowerDistribution.getInstance(1, ModuleType.kRev); // Enables power distribution logging
				break;
			case SIM:
				// logger.addDataReceiver(new WPILOGWriter(""));
				// logger.addDataReceiver(new NT4Publisher());
				break;
			case REPLAY:
				setUseTiming(false); // Run as fast as possible
				String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
				logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
				logger.addDataReceiver(
					new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))
				); // Save outputs to a new log
				break;
		}

		logger.start(); // Start logging
		robotContainer = new RobotContainer();
		timer.schedule(new LogSubsystemInputsTask(), 10, 20);
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like
	 * diagnostics that you want ran during disabled, autonomous, teleoperated and
	 * test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {}

	/** This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() {
		robotContainer.teleopInit();
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {}

	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {
		robotContainer.disabledInit();
	}

	/** This function is called periodically when disabled. */
	@Override
	public void disabledPeriodic() {}

	/** This function is called once when test mode is enabled. */
	@Override
	public void testInit() {}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}
}
