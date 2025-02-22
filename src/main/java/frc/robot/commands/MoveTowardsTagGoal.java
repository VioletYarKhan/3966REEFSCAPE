package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.GryphonLib.MovementCalculations;
import frc.robot.Constants.VisionConstants;

public class MoveTowardsTagGoal {
  
  private final Transform3d distances;
  private final double[] targetDistance;

  public MoveTowardsTagGoal(
        Transform3d distances, 
        double[] targetDistance) {
    this.distances = distances;
    this.targetDistance = targetDistance;;
    
  }

  public ChassisSpeeds getSpeeds() {
    if(distances.equals(new Transform3d())){
      return new ChassisSpeeds();
    }
    Transform3d camToTarget = distances;
    Transform3d robotToTarget = camToTarget.plus(VisionConstants.kRobotToCam);
    double omegaChange = 0;
    
    if (Units.radiansToDegrees(distances.getRotation().getZ()) < 0){
      omegaChange =  Units.radiansToDegrees(Math.PI - Math.abs(distances.getRotation().getZ()));
    } else {
      omegaChange = -(180 - Math.abs(Units.radiansToDegrees(distances.getRotation().getZ())));
    }
    

    double xSpeed = MovementCalculations.getInRangeRate(robotToTarget.getX(), targetDistance[0], 0.5);
    double ySpeed = MovementCalculations.getInRangeRate(robotToTarget.getY(), targetDistance[1], 0.5);
    double omegaSpeed = MovementCalculations.getTurnRate(omegaChange, 0.5) * Math.PI/2;

    if (omegaSpeed > 0.5){
      xSpeed *= 0.5;
      ySpeed *= 0.5;
    }

    SmartDashboard.putNumber("RotationSpeed", omegaSpeed);
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);


    ChassisSpeeds directedSpeeds = new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);
    SmartDashboard.putString("Speeds", directedSpeeds.toString());
    return directedSpeeds;
  }
}