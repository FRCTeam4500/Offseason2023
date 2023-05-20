package frc.robot;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import com.revrobotics.CANSparkMax;

import frc.robot.component.SparkMaxComponent;

public class Telemetry {
    
    private static Telemetry instanceTelemetry = null;

    Timer timer = new Timer();

    private Telemetry() {

    }

    public static synchronized Telemetry getInstance() {
        if (instanceTelemetry == null) {
            instanceTelemetry = new Telemetry();
        }
        return instanceTelemetry;
    }

    public void startPositionTelemetry(SparkMaxComponent motor, int timeFrame) {

        ArrayList<Double> positions = new ArrayList<Double>();

        TimerTask task = new TimerTask() {
            @Override
            public void run() {
                positions.add(motor.getAngle());
            }
        };

        timer.schedule(task, 0, timeFrame);

    }

}
