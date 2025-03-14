package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    TalonFX climber = new TalonFX(14);
    TalonFXConfiguration climbConfiguration = new TalonFXConfiguration();

    double targetReference;
    ControlType currentControlType;

    public Climber() {
        climbConfiguration.CurrentLimits.withSupplyCurrentLimit(80).withSupplyCurrentLimitEnable(true);
        climbConfiguration.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
                    
        climber.getConfigurator().apply(climbConfiguration);


        targetReference = 0;
        currentControlType = ControlType.kDutyCycle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wheel Position", getPosition());
        SmartDashboard.putString("Wheel Control Type", currentControlType.toString());
    }

    public void set(double speed) {
        climber.set(speed);
        currentControlType = ControlType.kDutyCycle;
    }

    public void setVelocity(double velocity) {
        climber.setControl(new VelocityDutyCycle(velocity));
        targetReference = velocity;
        currentControlType = ControlType.kVelocity;
    }

    public void setPosition(double position){
        SmartDashboard.putNumber("Requested Wheel Position", position);
        climber.setControl(new PositionDutyCycle(position));
        
        targetReference = position;
        currentControlType = ControlType.kPosition;
    }

    public void setVoltage(double voltage) {
        climber.setVoltage(voltage);
        currentControlType = ControlType.kVoltage;
    }

    public void setEncoderPosition(double position) {
        climber.setPosition(position);
    }

    public double getVelocity() {
        return climber.getVelocity().getValueAsDouble();
    }

    public double getPosition() {
        return climber.getPosition().getValueAsDouble();
    }

    public ControlType getControlType() {
        return currentControlType;
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

    public void climbCCW(){
        climber.set(-0.7);
    }
    public void climbCW(){
        climber.set(0.7);
    }
    public void stop(){
        climber.set(0);
    }
}
