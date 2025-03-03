package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    TalonFX m_climber = new TalonFX(14);

    public void climberCCW(){
        m_climber.set(0.5);
    }
    public void climberCW(){
        m_climber.set(-0.5);
    }
    public void climberStop(){
        m_climber.set(0);
    }
}
