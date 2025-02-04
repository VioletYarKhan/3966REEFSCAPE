package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToTag extends Command {
    private final PhotonPipelineResult photonRes;
    private final DriveSubsystem driveTrainSubsystem;

    private final Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(1.5, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(3, 8);
    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

    public AlignToTag(int target, PhotonPipelineResult photonRes, DriveSubsystem driveTrainSubsystem){
        this.driveTrainSubsystem = driveTrainSubsystem;
        this.photonRes = photonRes;
    }

    @Override
  public void execute() {
    var bestTarget = photonRes.getBestTarget();
    if (!bestTarget.equals(null)){
        Transform3d camToTarget = bestTarget.getBestCameraToTarget();
        Transform3d robotToTarget = camToTarget.plus(VisionConstants.kCamToRobot);
        Transform3d robotToGoal = robotToTarget.plus(TAG_TO_GOAL);

        double xSpeed = xController.calculate(robotToGoal.getX());
        double ySpeed = yController.calculate(robotToGoal.getY());
        double omegaSpeed = omegaController.calculate(robotToGoal.getRotation().getAngle());

        driveTrainSubsystem.driveRobotRelativeChassis(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, driveTrainSubsystem.getRotation()));
    }
  }
}
