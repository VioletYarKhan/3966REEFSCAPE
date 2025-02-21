package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class coralEffector extends SubsystemBase {
    SparkMax effectorWheel = new SparkMax(9, MotorType.kBrushless);

    public void outtake(){
        effectorWheel.set(0.5);
    }
    public void intake(){
        effectorWheel.set(-0.5);
    }
}
