package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralEffector extends SubsystemBase {
    
    TalonFX effectorWheel = new TalonFX(12);
    TalonFXConfiguration effectorConfig = new TalonFXConfiguration();

    double targetReference;
    ControlType currentControlType;

    public CoralEffector() {
        effectorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        effectorConfig.ClosedLoopRamps.withDutyCycleClosedLoopRampPeriod(0).withTorqueClosedLoopRampPeriod(0).withVoltageClosedLoopRampPeriod(0);
        effectorConfig.OpenLoopRamps.withDutyCycleOpenLoopRampPeriod(0).withTorqueOpenLoopRampPeriod(0).withVoltageOpenLoopRampPeriod(0);
        effectorConfig.Slot0.withKP(0.5);
                    
        effectorWheel.getConfigurator().apply(effectorConfig);


        targetReference = 0;
        currentControlType = ControlType.kDutyCycle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wheel Position", getPosition());
    }

    public void set(double speed) {
        effectorWheel.set(speed);
        currentControlType = ControlType.kDutyCycle;
    }

    public void setVelocity(double velocity) {
        effectorWheel.setControl(new VelocityDutyCycle(velocity));
        targetReference = velocity;
        currentControlType = ControlType.kVelocity;
    }

    public void setPosition(double position){
        SmartDashboard.putNumber("Requested Wheel Position", position);
        effectorWheel.setControl(new PositionDutyCycle(position));
        targetReference = position;
        currentControlType = ControlType.kPosition;
    }

    public void setVoltage(double voltage) {
        effectorWheel.setVoltage(voltage);
        currentControlType = ControlType.kVoltage;
    }

    public void setEncoderPosition(double position) {
        effectorWheel.setPosition(position);
    }

    public double getVelocity() {
        return effectorWheel.getVelocity().getValueAsDouble();
    }

    public double getPosition() {
        return effectorWheel.getPosition().getValueAsDouble();
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


    public void outtake(){
        set(-0.15);
    }
    public void intake(){
        set(0.15);
    }
    public void stop(){
        set(0);
    }

    public void goToPosition(int level){
        if (level == 4){
            setPosition(getPosition()-0.95);
        } else {
            setPosition(getPosition());
        }
    }
}