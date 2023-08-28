package frc.robot.commands.complexCommands;

import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystem.swerve.SwerveDrive;

public class SwerveOrchestraCommand extends CommandBase {

	SwerveDrive swerve;
	CommandJoystick joystick;

	Orchestra orchestra;

	/* An array of songs that are available to be played, can you guess the song/artists? */
	String[] songs = new String[] {
		"song1.chrp",
		"song2.chrp",
		"song3.chrp",
		"song4.chrp",
		"song5.chrp",
		"song6.chrp",
		"song7.chrp",
		"song8.chrp",
		"song9.chrp",/* the remaining songs play better with three or more FXs */
		"song10.chrp",
		"song11.chrp",
	};

	/* track which song is selected for play */
	int songSelection = 0;

	/* overlapped actions */
	int timeToPlayLoops = 0;

	boolean activate = false;

	public SwerveOrchestraCommand(
		SwerveDrive swerve,
		CommandJoystick joystick
	) {
		this.swerve = swerve;
		this.joystick = joystick;
		this.orchestra = new Orchestra(this.swerve.getTalons());

		addRequirements(swerve);
	}

	private void loadSelectedSong(int offset) {
		/* increment song selection */
		songSelection += offset;
		/* wrap song index in case it exceeds boundary */
		if (songSelection >= songs.length) {
			songSelection = 0;
		}
		if (songSelection < 0) {
			songSelection = songs.length - 1;
		}

		/* load the chirp file */
		orchestra.loadMusic(songs[songSelection]);

		/* schedule a play request, after a delay.  
           This gives the Orchestra service time to parse chirp file.
           If play() is called immedietely after, you may get an invalid action error code. */
		timeToPlayLoops = 10;
	}

	public void activate() {
		activate = !activate;
	}

	@Override
	public void initialize() {
		loadSelectedSong(0);
	}

	@Override
	public void execute() {
		/* if song selection changed, auto-play it */
		if (timeToPlayLoops > 0 && activate) {
			--timeToPlayLoops;
			if (timeToPlayLoops == 0) {
				/* scheduled play request */
				orchestra.play();
			}
		}
	}
}
