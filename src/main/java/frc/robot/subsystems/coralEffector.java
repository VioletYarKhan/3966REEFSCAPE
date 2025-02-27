package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class coralEffector extends SubsystemBase {
    Talon effectorWheel = new Talon(1);

    public void outtake(){
        effectorWheel.set(0.5);
    }
    public void intake(){
        effectorWheel.set(-0.5);
    }
    public void stop(){
        effectorWheel.set(0);
    }
}
