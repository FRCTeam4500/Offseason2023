package frc.robot.subsystem.messaging;

import java.util.ArrayList;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MessagingSystem extends SubsystemBase {
    private static MessagingSystem systemInstance;
    private ArrayList<String> messages;
    private Timer timer = new Timer();

    private MessagingSystem() {
        messages = new ArrayList<String>();
        timer.start();
    }

    public void addMessage(String message) {
        messages.add(message);
    }

    public static synchronized MessagingSystem getInstance() {
		if (systemInstance == null) {
			systemInstance = new MessagingSystem();
		}
		return systemInstance;
	}

    public void initSendable(SendableBuilder builder) {
        for (int i = 0; i < messages.size(); i++) {
            final int iV2 = i; // The get method for ArrayList only works with local variables if they are final
            Shuffleboard.getTab("Messaging System").addString("Messages", () -> "Time: " + timer.get() + " | " + messages.get(iV2));
        }
    }
}
