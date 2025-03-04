package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class coralEffector extends SubsystemBase {
    TalonFX effectorWheel = new TalonFX(12);

    public void outtake(){
        effectorWheel.set(-0.1);
    }
    public void intake(){
        effectorWheel.set(0.1);
    }
    public void stop(){
        effectorWheel.set(0);
    }
}
