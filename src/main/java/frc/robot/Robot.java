// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.utilities.LogSubsystemInputsTask;

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
	private Timer timer;

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

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run(); // Runs the scheduler, which is what runs all commands and subsystems
	}

	@Override
	public void autonomousInit() {
		robotContainer.autonomousInit();
	}
	
	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		robotContainer.teleopInit();
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void disabledInit() {
		robotContainer.disabledInit();
	}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void testInit() {}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
