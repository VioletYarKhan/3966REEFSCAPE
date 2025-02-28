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

public class effectorWrist extends SubsystemBase {
    SparkMax wristMotor = new SparkMax(11, MotorType.kBrushless);
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    RelativeEncoder wristEncoder = wristMotor.getEncoder();
    SparkClosedLoopController pid;

    double targetReference;
    ControlType currentControlType;

    public effectorWrist() {
        wristConfig.idleMode(IdleMode.kBrake).inverted(true).openLoopRampRate(0).closedLoopRampRate(0).closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.05, 0, 0).minOutput(-0.4).maxOutput(0.6);
        wristConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        wristMotor.configure(wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        pid = wristMotor.getClosedLoopController();

        targetReference = 0;
        currentControlType = ControlType.kDutyCycle;
    }

    public void set(double speed) {
        wristMotor.set(speed);
        currentControlType = ControlType.kDutyCycle;
    }

    public void setVelocity(double velocity) {
        pid.setReference(velocity, ControlType.kVelocity);

        targetReference = velocity;
        currentControlType = ControlType.kVelocity;
    }

    public void setPosition(double position) {
        pid.setReference(position, ControlType.kPosition);

        targetReference = position;
        currentControlType = ControlType.kPosition;
    }

    public void setVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
        
        currentControlType = ControlType.kVoltage;
    }

    public void setEncoderPosition(double position) {
        wristEncoder.setPosition(position);
    }

    public double getVelocity() {
        return wristEncoder.getVelocity();
    }

    public double getPosition() {        
        return wristEncoder.getPosition();
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