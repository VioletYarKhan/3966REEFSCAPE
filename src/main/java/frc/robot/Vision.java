package frc.robot;

import static frc.robot.Constants.VisionConstants.kCamToRobot;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private static PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraName);
    private static PhotonPipelineResult result;
    private static PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.LOWEST_AMBIGUITY, VisionConstants.kRobotToCam);

    public void periodic() {
        result = camera.getLatestResult();
        if (result.hasTargets()){SmartDashboard.putNumber("Best Tag Seen", result.getBestTarget().getFiducialId());}
        else{SmartDashboard.putNumber("Best Tag Seen", 0);}
    }

    public static PhotonPipelineResult getResult(){
        return result;
    }

    public static int getBestTag(){
        try{
        return result.getBestTarget().fiducialId;
        } catch (NullPointerException e){
            return 0;
        }
    }

    public static PhotonCamera getCamera(){
        return camera;
    }

    public static boolean resultHasTargets(){
        return result.hasTargets();
    }

    public static int[] tagsInFrame(){
        int[] tags = new int[0];
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            tags = new int[result.getTargets().toArray().length];
            int i = 0;
            for (var target : result.getTargets()) {
                tags[i] = target.getFiducialId();
                i++;
            }
        }
        return tags;
    }

    public static EstimatedRobotPose getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult result, SwerveDrivePoseEstimator poseEst) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        var update = photonPoseEstimator.update(result);
        Transform3d botToTarget = result.getBestTarget().getBestCameraToTarget().plus(kCamToRobot);
        poseEst.setVisionMeasurementStdDevs(VecBuilder.fill(botToTarget.getX()/2.5, botToTarget.getY()/2.5, Units.degreesToRadians(30)));
        Pose3d currentPose3d = update.get().estimatedPose;
        double photonTimestamp = update.get().timestampSeconds;
        
        return new EstimatedRobotPose(currentPose3d, photonTimestamp, result.getTargets(), PoseStrategy.LOWEST_AMBIGUITY);
    }


    public static double targetYaw(int targetNumber){
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == targetNumber) {
                    // Found Tag, record its information
                    return target.getYaw();
                }
            }
        }
        return 0;
    }


    public static Transform3d targetTransform(int targetNumber){
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == targetNumber) {
                    return target.getBestCameraToTarget();
                }
            }
        }
        return new Transform3d();
    }

    public static PhotonTrackedTarget returnTag (int targetNumber){
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == targetNumber) {
                    return target;
                }
            }
        }
        return new PhotonTrackedTarget();
    }
}
