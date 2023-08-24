package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.component.TalonFXComponent;
import java.util.ArrayList;

public class TalonFXOrchestra extends SubsystemBase {

	Orchestra orchestra;

	TalonFXComponent[] controllers;

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

	/* Joystick, can move back to localize everything */
	OperatorController instanceController = OperatorController.getInstance();

	public final Trigger nextSongButton = instanceController.pov(90);
	public final Trigger prevSongButton = instanceController.pov(270);
	public final Trigger playPauseButton = instanceController.pov(0);

	TalonFXOrchestra(TalonFXComponent[] controllers) {
		this.controllers = controllers;
		createInstruments();
		loadSelectedSong(0);
		setPOVButtons();
	}

	private void setPOVButtons() {
		nextSongButton.toggleOnTrue(
			new InstantCommand(() -> loadSelectedSong(1))
		);
		prevSongButton.toggleOnTrue(
			new InstantCommand(() -> loadSelectedSong(-1))
		);
		playPauseButton
			.and(() -> orchestra.isPlaying())
			.toggleOnTrue(new InstantCommand(() -> orchestra.pause()))
			.toggleOnFalse(
				new InstantCommand(() -> orchestra.play()) // TODO: Check if this does what I think it does...
			);
	}

	private void createInstruments() {
		/* A list of TalonFX's that are to be used as instruments */
		ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();

		/* Initialize the TalonFX's to be used */
		for (int i = 0; i < controllers.length; ++i) {
			instruments.add(controllers[i]);
		}

		orchestra = new Orchestra(instruments);
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

	@Override
	public void periodic() {
		/* if song selection changed, auto-play it */
		if (timeToPlayLoops > 0) {
			--timeToPlayLoops;
			if (timeToPlayLoops == 0) {
				/* scheduled play request */
				orchestra.play();
			}
		}
	}
}
