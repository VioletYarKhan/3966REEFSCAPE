package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class coralFunnel extends SubsystemBase {
    SparkMax funnelMotor = new SparkMax(13, MotorType.kBrushless);
    SparkMaxConfig funnelConfig = new SparkMaxConfig();
    RelativeEncoder funnelRelativeEncoder = funnelMotor.getAlternateEncoder();
    SparkClosedLoopController pid;

    double targetReference;
    ControlType currentControlType;

    public coralFunnel() {
        funnelConfig.idleMode(IdleMode.kBrake).inverted(false).openLoopRampRate(0).closedLoopRampRate(0).closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.05, 0, 0).minOutput(-0.2).maxOutput(1);
        funnelConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        funnelMotor.configure(funnelConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        pid = funnelMotor.getClosedLoopController();

        targetReference = 0;
        currentControlType = ControlType.kDutyCycle;
    }

    public void set(double speed) {
        funnelMotor.set(speed);
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
        funnelMotor.setVoltage(voltage);
        currentControlType = ControlType.kVoltage;
    }

    public void setEncoderPosition(double position) {
        funnelRelativeEncoder.setPosition(position);
    }

    public double getVelocity() {
        return funnelRelativeEncoder.getVelocity();
    }

    public double getPosition() {
        return funnelRelativeEncoder.getPosition();
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
