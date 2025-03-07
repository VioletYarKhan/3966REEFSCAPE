package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    TalonFX climber = new TalonFX(14);

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
