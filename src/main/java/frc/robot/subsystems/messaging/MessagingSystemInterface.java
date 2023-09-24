package frc.robot.subsystems.messaging;

import org.littletonrobotics.junction.AutoLog;

public interface MessagingSystemInterface {
    @AutoLog
    public class MessagingSystemInputs {
        public String messageString = "";

        
    }
    public default void updateInputs(MessagingSystemInputs inputs) {}
}
