package frc.robot.subsystem.messaging;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MessagingSystem extends SubsystemBase implements MessagingSystemInterface{
    private static MessagingSystem systemInstance;
    private String message;
    private Timer timer = new Timer();

    private MessagingSystemInputsAutoLogged inputs = new MessagingSystemInputsAutoLogged();

    public MessagingSystemInputsAutoLogged getInputs() {
        return inputs;
    }

    private MessagingSystem() {
        message = "        TIME | MESSAGE";
        timer.start();
    }

    public void addMessage(String message) {
        this.message = this.message + "\n" + "Time: " + (double) Math.round(timer.get() * 100) / 100 + " | " + message;
    }

    public static synchronized MessagingSystem getInstance() {
		if (systemInstance == null) {
			systemInstance = new MessagingSystem();
		}
		return systemInstance;
	}

    public void updateInputs(MessagingSystemInputs inputs) {
        inputs.messageString = message;
    }

    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Messages", () -> message, null);
    }
}
