package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GryphonLib.MovementCalculations;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class MoveTowardsTagGoal extends Command {
  
  private final Transform3d distances;
  private final double[] thresholds;
  private final DriveSubsystem driveSubsystem;
  private final Transform2d distancesFromTarget;
  private final boolean rotateTowards;

  public MoveTowardsTagGoal(
        Transform3d distances, 
        double[] thresholds,
        DriveSubsystem driveSubsystem,
        Transform2d distancesFromTarget,
        boolean rotateTowards
        ) {
    this.distances = distances;
    this.thresholds = thresholds;
    this.driveSubsystem = driveSubsystem;
    this.distancesFromTarget = distancesFromTarget;
    this.rotateTowards = rotateTowards;
    
    addRequirements(driveSubsystem);
  }
  @Override
  public void execute() {
    ChassisSpeeds directedSpeeds = new ChassisSpeeds();
    if(!distances.equals(new Transform3d())){
      Transform3d camToTarget = distances;
      Transform3d robotToTarget = camToTarget.plus(VisionConstants.kRobotToCam);
      double omegaChange = 0;
      if (rotateTowards){
        if (Units.radiansToDegrees(distances.getRotation().getZ()) < 0){
          omegaChange =  Units.radiansToDegrees(Math.PI - Math.abs(distances.getRotation().getZ()));
        } else {
          omegaChange = -(180 - Math.abs(Units.radiansToDegrees(distances.getRotation().getZ())));
        }
      }

      double xSpeed = MovementCalculations.getInRangeRate(robotToTarget.getX(), distancesFromTarget.getX(), 0.5);
      double ySpeed = MovementCalculations.getInRangeRate(robotToTarget.getY(), distancesFromTarget.getY(), 0.5);
      double omegaSpeed = MovementCalculations.getTurnRate(omegaChange, 0.5) * Math.PI/2;

      if (omegaSpeed > 0.5){
        xSpeed *= 0.5;
        ySpeed *= 0.5;
      }

      SmartDashboard.putNumber("RotationSpeed", omegaSpeed);
      SmartDashboard.putNumber("xSpeed", xSpeed);
      SmartDashboard.putNumber("ySpeed", ySpeed);


      directedSpeeds = new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);
    }
    SmartDashboard.putString("Speeds", directedSpeeds.toString());
    driveSubsystem.driveRobotRelativeChassis(directedSpeeds);
  }

  @Override
  public boolean isFinished(){
    if (rotateTowards){
      return (Math.abs(distances.getX()) < thresholds[0] && Math.abs(distances.getY()) < thresholds[1] && Math.abs(distances.getRotation().getZ()) < thresholds[2]);
    } else {
      return (Math.abs(distances.getX()) < thresholds[0] && Math.abs(distances.getY()) < thresholds[1]);
    }
  }
}