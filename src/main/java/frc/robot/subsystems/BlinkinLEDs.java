package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;

public class BlinkinLEDs extends SubsystemBase {
    public Spark m_blinkin = new Spark(0);

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Blinkin Color", m_blinkin.get());
    }

    public BlinkinLEDs(){
        m_blinkin.set(BlinkinConstants.green);
    }
    

    public void set(double value){
        m_blinkin.set(value);
    }

    public void setReadyIntake(){
        m_blinkin.set(BlinkinConstants.blue);
    }

    public void setHasCoral(){
        m_blinkin.set(BlinkinConstants.green);
    }

    public double getColor(){
        return m_blinkin.get();
    }
}
