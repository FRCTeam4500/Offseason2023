package frc.robot.subsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.component.SparkMaxComponent;
import frc.robot.component.TalonSRXComponent;

import java.util.function.DoubleSupplier;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private SparkMaxComponent tiltMotor;
    private TalonSRXComponent winchMotor;
    private SparkMaxPIDController tiltPIDController;
    private double targetTiltAngle;
    private double targetWinchPosition;

    /**
     * Creates a new Arm from Constants
     */
    public Arm() {
        tiltMotor = new SparkMaxComponent(ArmConstants.TILT_MOTOR_ID, ArmConstants.TILT_MOTOR_TYPE);
        winchMotor = new TalonSRXComponent(ArmConstants.WINCH_MOTOR_ID);

        tiltMotor.setInverted(true);
        
        tiltPIDController = tiltMotor.getPIDController();
        tiltPIDController.setP(0.04);
        tiltPIDController.setI(0);
        tiltPIDController.setD(0);
        tiltPIDController.setOutputRange(-.5, .5);

        winchMotor.config_kP(0, .4);
        winchMotor.config_kI(0, 0);
        winchMotor.config_kD(0, 0);
        winchMotor.configAllowableClosedloopError(0, 400);
        winchMotor.configForwardSoftLimitEnable(true);
        winchMotor.configForwardSoftLimitThreshold(10000);
        winchMotor.configPeakOutputForward(.6);
        winchMotor.configPeakOutputReverse(-0.3);

    }
    
    public void setTilt(double position) {
        targetTiltAngle = position;
        tiltMotor.setAngle(position);
    }

    public void changeTilt(double addition) {
        setTilt(getTilt().getAsDouble() + addition);;
    }

    public DoubleSupplier getTilt() {
        return () -> tiltMotor.getAngle();
    }

    public DoubleSupplier getTargetTilt() {
        return () -> targetTiltAngle;
    }

    public void setWinchPosition(double position) {
        targetWinchPosition = position;
        winchMotor.setAngle(position);
    }

    public void changeWinchPosition(double addition) {
        setWinchPosition(getWinchPosition().getAsDouble() + addition);
    }

    public DoubleSupplier getWinchPosition() {
        return () -> winchMotor.getSelectedSensorPosition();
    }

    public DoubleSupplier getTargetWinchPosition() {
        return () -> targetWinchPosition;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target tilt position", getTargetTilt(), null);
        builder.addDoubleProperty("Target winch output", getTargetWinchPosition(), null);
    
        builder.addDoubleProperty("Current Encoder Tilt Position", getTilt(), null);
        builder.addDoubleProperty("Current Winch Encoder Position", getWinchPosition(), null);
    }
}
