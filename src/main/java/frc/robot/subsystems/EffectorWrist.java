package frc.robot.subsystems;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.WristConstants;

public class EffectorWrist extends SubsystemBase {
    SparkFlex wristMotor = new SparkFlex(11, MotorType.kBrushless);
    RelativeEncoder wristEncoder = wristMotor.getEncoder();
    SparkClosedLoopController pid;

    double targetReference;
    ControlType currentControlType;

    public EffectorWrist() {
        SmartDashboard.putNumber("Wrist Intake Angle", WristConstants.IntakeAngle);
        SmartDashboard.putNumber("Wrist L1 Angle", WristConstants.L1Angle);
        SmartDashboard.putNumber("Wrist L2-3 Angle", WristConstants.L2_3Angle);
        SmartDashboard.putNumber("Wrist L4 Angle", WristConstants.L4Angle);
        wristMotor.configure(Configs.Wrist.wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        pid = wristMotor.getClosedLoopController();

        targetReference = 0;
        currentControlType = ControlType.kDutyCycle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
        SmartDashboard.putNumber("Wrist Velocity", wristEncoder.getVelocity());
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
        SmartDashboard.putNumber("Requested Wrist Position", position);

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