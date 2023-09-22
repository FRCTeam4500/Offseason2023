package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestPath1 extends CommandBase {

	public static SequentialCommandGroup getPathCommand() {
		return new SequentialCommandGroup();
	}
}
