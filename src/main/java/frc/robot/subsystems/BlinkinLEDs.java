package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;

public class BlinkinLEDs extends SubsystemBase {
    public Spark m_blinkin = new Spark(1);

    public BlinkinLEDs(){
        m_blinkin.set(0.01);
    }
    

    public void set(double value){
        m_blinkin.set(value);
    }

    public void setReadyIntake(){
        m_blinkin.set(BlinkinConstants.black);
    }

    public void setHasCoral(){
        m_blinkin.set(BlinkinConstants.Larson_Scanner_1);
    }

    public double getColor(){
        return m_blinkin.get();
    }
}
