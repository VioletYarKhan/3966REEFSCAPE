package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    TalonFX climber = new TalonFX(14);
    TalonFXConfiguration climbConfiguration = new TalonFXConfiguration();

    public Climber(){
        climbConfiguration.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
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
