package frc.robot.subsystems.Wrist;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface WristIO {
        public void periodic();
    
        public void set(double speed);
    
        public void setVelocity(double velocity);
    
        public void setPosition(double position);
    
        public void setVoltage(double voltage);
    
        public void setEncoderPosition(double position);
    
        public double getVelocity();
    
        public double getPosition();
    
        public boolean atTarget(double threshold);

        public SubsystemBase returnSubsystem();
               
        public Optional<MechanismLigament2d> returnLigament();
    }
