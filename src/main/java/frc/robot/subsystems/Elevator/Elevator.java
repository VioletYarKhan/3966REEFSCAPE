package frc.robot.subsystems.Elevator;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import java.util.Optional;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase implements ElevatorIO {
    SparkMax elevatorL = new SparkMax(10, MotorType.kBrushless);
    SparkMax elevatorR = new SparkMax(9, MotorType.kBrushless);
    RelativeEncoder encoderL = elevatorL.getEncoder();
    RelativeEncoder encoderR = elevatorR.getEncoder();
    SparkClosedLoopController pid;

    double targetReference;
    ControlType currentControlType;

    private final Mechanism2d elevatorSimMechanism = new Mechanism2d(Units.inchesToMeters(10), Units.inchesToMeters(70));
    private final MechanismRoot2d elevatorRoot = elevatorSimMechanism.getRoot("Base", Units.inchesToMeters(15), Units.inchesToMeters(8));
    public final MechanismLigament2d elevatorLigament = elevatorRoot.append(new MechanismLigament2d("Elevator", Units.inchesToMeters(10), 90, 10, new Color8Bit(Color.kBlue)));

    public Elevator() {
        SmartDashboard.putData("Elevator Mech2D", elevatorSimMechanism);
        SmartDashboard.putNumber("Elevator Intake Height", ElevatorConstants.IntakeHeight);
        SmartDashboard.putNumber("Elevator L1 Height", ElevatorConstants.L1Height);
        SmartDashboard.putNumber("Elevator L2 Height", ElevatorConstants.L2Height);
        SmartDashboard.putNumber("Elevator L3 Height", ElevatorConstants.L3Height);
        SmartDashboard.putNumber("Elevator L4 Height", ElevatorConstants.L4Height);
        elevatorL.configure(Configs.Elevator.elevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        elevatorR.configure(Configs.Elevator.elevatorFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        pid = elevatorL.getClosedLoopController();

        targetReference = 0;
        currentControlType = ControlType.kDutyCycle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putString("Elevator Control Type", currentControlType.toString());
        SmartDashboard.putNumber("Elevator L Position", encoderL.getPosition());
        SmartDashboard.putNumber("Elevator R Position", encoderR.getPosition());
        elevatorLigament.setLength(getPosition() * Units.inchesToMeters(70)/22 + 0.3);
    }

    public void set(double speed) {
        elevatorL.set(speed);
        currentControlType = ControlType.kDutyCycle;
    }

    public void setVelocity(double velocity) {
        pid.setReference(velocity, ControlType.kVelocity);

        targetReference = velocity;
        currentControlType = ControlType.kVelocity;
    }

    public void setPosition(double position) {
        SmartDashboard.putNumber("Requested Elevator Position", position);
        pid.setReference(position, ControlType.kPosition);

        targetReference = position;
        currentControlType = ControlType.kPosition;
    }

    public void setVoltage(double voltage) {
        elevatorL.setVoltage(voltage);
        
        currentControlType = ControlType.kVoltage;
    }

    public void setEncoderPosition(double position) {
        encoderL.setPosition(position);
        encoderR.setPosition(position);
    }

    public double getVelocity() {
        double velocity = 0;
        velocity += encoderL.getVelocity();
        velocity += encoderR.getVelocity();
        return velocity / 2;
    }

    public double getPosition() {
        double position = 0;
        
        position += encoderL.getPosition();
        position += encoderR.getPosition();
        
        return position / 2;
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
    
    @Override
    public SubsystemBase returnSubsystem(){
        return this;
    }

    @Override
    public Optional<MechanismLigament2d> returnLigament() {
        return null;
    }
}