package frc.robot.subsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.component.SparkMaxComponent;
import frc.robot.component.TalonSRXComponent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;


public class Arm extends SubsystemBase {
    public SparkMaxComponent tiltMotor;
    private TalonSRXComponent winchMotor;
    private SparkMaxPIDController tiltPIDController;
    private double targetTiltAngle;
    private double targetWinchPosition;

    /**
     * Creates a new Arm from Constants
     */
    public Arm() {
        this.tiltMotor = new SparkMaxComponent(ArmConstants.TILT_MOTOR_ID, ArmConstants.TILT_MOTOR_TYPE);
        this.winchMotor = new TalonSRXComponent(ArmConstants.WINCH_MOTOR_ID);

        this.tiltMotor.setInverted(true);
        
        this.tiltPIDController = this.tiltMotor.getPIDController();
        this.tiltPIDController.setP(0.04);
        this.tiltPIDController.setI(0);
        this.tiltPIDController.setD(0);

        this.winchMotor.config_kP(0, .4);
        this.winchMotor.config_kI(0, 0);
        this.winchMotor.config_kD(0, 0);

        this.tiltPIDController.setOutputRange(-.5, .5);

        this.winchMotor.configAllowableClosedloopError(0, 400);
        this.winchMotor.configForwardSoftLimitEnable(true);
        this.winchMotor.configForwardSoftLimitThreshold(10000);
        this.winchMotor.configPeakOutputForward(.6);
        this.winchMotor.configPeakOutputReverse(-0.3);

    }
    
    /**
     * Creates a new Arm from Override
     * @param tiltMotor
     * @param winchMotor
     */
    public Arm(SparkMaxComponent tiltMotor, TalonSRXComponent winchMotor) {
        this.tiltMotor = tiltMotor;
        this.winchMotor = winchMotor;
    }

    /**
     * Sets the position of the tilt motor.
     * 
     * @param position 
     */
    public void setTilt(double position) {
        targetTiltAngle = position;
        tiltMotor.setAngle(position);
    }

    public double getTilt() {
        return tiltMotor.getAngle();
    }

    /**
     * Sets the position of the winch motor. "position" is not speed, it is motor rotations
     * @param position is the angle that it has to turn
     */
    public void setWinch(double position) {
        targetWinchPosition = position;
        winchMotor.setAngle(position);
    }

    public void setOutput(double output) {
        tiltMotor.setOutput(output);
    }

    public void zero(){
        tiltMotor.getEncoder().setPosition(0);
    }

    public double getWinchPosition() {
        return winchMotor.getSelectedSensorPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target tilt position", () -> targetTiltAngle, null);
        builder.addDoubleProperty("Target winch output", () -> targetWinchPosition, null);
    
        builder.addDoubleProperty("Current Encoder Tilt Position", () -> tiltMotor.getEncoder().getPosition(), null);
        builder.addDoubleProperty("Current Winch Encoder Position", () -> winchMotor.getSelectedSensorPosition(), null);
    }
}
