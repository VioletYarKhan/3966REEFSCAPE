package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GryphonLib.MovementCalculations;
import frc.robot.Vision;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class MoveTowardsTagGoal extends Command {

  private final DriveSubsystem driveSubsystem;
  private final double[] thresholds;
  private final int tag;
  private Transform3d distances;
  private double omegaChange;

  public MoveTowardsTagGoal(
        DriveSubsystem driveSubsystem,
        double[] thresholds,
        int tag) {
    this.driveSubsystem = driveSubsystem;
    this.thresholds = thresholds;
    this.tag = tag;
    distances = Vision.targetTransform(tag);
  }

  @Override
  public void execute() {
    distances = Vision.targetTransform(tag);
    ChassisSpeeds directedSpeeds = new ChassisSpeeds();
    if(!distances.equals(new Transform3d())){
      
    
      Transform3d camToTarget = distances;
      Transform3d robotToTarget = camToTarget.plus(VisionConstants.kRobotToCam);
      omegaChange = 0;
      
      if (Units.radiansToDegrees(distances.getRotation().getZ()) < 0){
        omegaChange =  Units.radiansToDegrees(Math.PI - Math.abs(distances.getRotation().getZ()));
      } else {
        omegaChange = -(180 - Math.abs(Units.radiansToDegrees(distances.getRotation().getZ())));
      }
      

      double xSpeed = MovementCalculations.getInRangeRate(robotToTarget.getX(), distances.getX(), 0.5);
      double ySpeed = MovementCalculations.getInRangeRate(robotToTarget.getY(), distances.getY(), 0.5);
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
  public boolean isFinished() {
    return (Math.abs(distances.getX()) < thresholds[0] && Math.abs(distances.getY()) < thresholds[1] && Math.abs(omegaChange) < thresholds[2]);
  }
}