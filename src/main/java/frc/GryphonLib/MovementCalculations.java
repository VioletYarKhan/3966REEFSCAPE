package frc.GryphonLib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MovementCalculations {
    public static double getTurnRate(double yawChange, double maxSpeed){
        double turn = 0.0;
        if (Math.abs(yawChange) > 8){
            if (yawChange > 0){
            turn = Math.min(yawChange/30, maxSpeed);
            } else{
            turn = Math.max(yawChange/30, -maxSpeed);
            }
        }
    return turn;
    }
    
    public static double getInRangeRate(double distance, double desiredDistance, double maxSpeed){
        double speed = 0.0;
        double wantedDist = desiredDistance;
        double distChange = wantedDist-distance;
        SmartDashboard.putNumber("DistChange", distChange);
        if (Math.abs(distChange) > 0.1){
            if (distChange > 0){
                speed = Math.min(distChange, maxSpeed);
            } else{
                speed = Math.max(distChange, -maxSpeed);
            }
        }
    return speed;
    }
}
