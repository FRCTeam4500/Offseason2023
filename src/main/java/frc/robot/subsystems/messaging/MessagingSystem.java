package frc.robot.subsystems.messaging;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MessagingSystem extends SubsystemBase implements MessagingSystemInterface{
    private static MessagingSystem systemInstance;
    private String message;
    private String newestMessage;
    private boolean isEnabled = false;

    private MessagingSystemInputsAutoLogged inputs = new MessagingSystemInputsAutoLogged();

    public MessagingSystemInputsAutoLogged getInputs() {
        return inputs;
    }

    private MessagingSystem() {
        message = "MESSAGES APPEAR BELOW";
    }

    public void addMessage(String message) {
        if(isEnabled) {
            newestMessage = message;
            this.message = this.message + "\n" + newestMessage;
        }
    }

    public void enableMessaging(boolean enable) {
        isEnabled = enable;
    }

    public void enableMessaging() {
        isEnabled = true;
    }

    public static synchronized MessagingSystem getInstance() {
		if (systemInstance == null) {
			systemInstance = new MessagingSystem();
		}
		return systemInstance;
	}

    public void updateInputs(MessagingSystemInputs inputs) {
        inputs.messageString = newestMessage;
    }

    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Messages", () -> message, null);
    }
}
