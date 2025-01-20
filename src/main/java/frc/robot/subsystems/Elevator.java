package frc.robot.subsystems;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    double targetReference;
    ControlType currentControlType;
    private SparkMax[] motors = new SparkMax[4];
    private RelativeEncoder[] encoders = new RelativeEncoder[4];
    private SparkMaxConfig[] configs = new SparkMaxConfig[4];
    private SparkClosedLoopController[] pids = new SparkClosedLoopController[4];
    private int[] ids = new int[4];

    public Elevator() {
        motors = new SparkMax[ids.length];
        encoders = new RelativeEncoder[ids.length];
        configs = new SparkMaxConfig[ids.length];
        pids = new SparkClosedLoopController[ids.length];
        

        for (int i = 0; i < ids.length; i++) {
            motors[i] = new SparkMax(ids[i], MotorType.kBrushless);
            configs[i] = new SparkMaxConfig();
            configs[i].idleMode(IdleMode.kBrake).openLoopRampRate(0).closedLoopRampRate(0);
            encoders[i] = motors[i].getEncoder();
            configs[i].closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.05, 0, 0).minOutput(2).maxOutput(-0.1);
            configs[i].encoder.positionConversionFactor(1).velocityConversionFactor(1);
            motors[i].configure(configs[i], ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            pids[i] = motors[i].getClosedLoopController();
            
        }

        targetReference = 0;
        currentControlType = ControlType.kDutyCycle;
    }

    public void set(double speed) {
        for (SparkMax motor : motors) {
            motor.set(speed);
        }
        currentControlType = ControlType.kDutyCycle;
    }

    public void setVelocity(double velocity) {
        for (SparkClosedLoopController pid : pids) {
            pid.setReference(velocity, ControlType.kVelocity);
        }

        targetReference = velocity;
        currentControlType = ControlType.kVelocity;
    }

    public void setPosition(double position) {
        for (SparkClosedLoopController pid : pids) {
            pid.setReference(position, ControlType.kPosition);
        }

        targetReference = position;
        currentControlType = ControlType.kPosition;
    }

    public void setVoltage(double voltage) {
        for (SparkMax motor : motors) {
            motor.setVoltage(voltage);
        }
        currentControlType = ControlType.kVoltage;
    }

    public void setEncoderPosition(double position) {
        for (RelativeEncoder encoder: encoders) {
            encoder.setPosition(position);
        }
    }

    public double getVelocity() {
        double velocity = 0;
        for (RelativeEncoder encoder : encoders) {
            velocity += encoder.getVelocity();
        }
        return velocity / encoders.length;
    }

    public double getPosition() {
        double position = 0;
        for (RelativeEncoder encoder : encoders) {
            position += encoder.getPosition();
        }
        return position / encoders.length;
    }

    public boolean atTarget(double threshold) {
        if (currentControlType == ControlType.kVelocity) {
            return Math.abs(getVelocity() - targetReference) < threshold;
        } else if (currentControlType == ControlType.kPosition) {
            return Math.abs(getPosition() - targetReference) < threshold;
        } else {
            return false;
        }
    }
  
}