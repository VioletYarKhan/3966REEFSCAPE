package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraName);
    private PhotonPipelineResult result;
    private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, VisionConstants.kRobotToCam);
    private int[] tags;

    public void periodic() {
        result = camera.getLatestResult();
        if (result.hasTargets()){SmartDashboard.putNumber("Best Tag Seen", result.getBestTarget().getFiducialId());}
        else{SmartDashboard.putNumber("Best Tag Seen", 0);}
    }

    public PhotonPipelineResult getResult(){
        return result;
    }

    public int getBestTag(){
        return result.getBestTarget().fiducialId;
    }

    public PhotonCamera getCamera(){
        return camera;
    }

    public boolean resultHasTargets(){
        return result.hasTargets();
    }

    public int[] tagsInFrame(){
        tags = new int[0];
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

    public Pose2d getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        Pose2d botPose = prevEstimatedRobotPose;
        if (result.hasTargets()) {
            var update = photonPoseEstimator.update(result);
            Pose3d currentPose3d = update.get().estimatedPose;
            botPose = currentPose3d.toPose2d();
            // double photonTimestamp = update.get().timestampSeconds;
        }
        return botPose;
    }


    public double targetYaw(int targetNumber){
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


    public double[] targetDistance(int targetNumber){
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == targetNumber) {
                    // Found Tag, record its information
                    double targetRangeX = target.getBestCameraToTarget().getX();
                    double targetRangeY = target.getBestCameraToTarget().getY();
                    double targetRangeOmega = target.getBestCameraToTarget().getRotation().getAngle();
                    return new double[]{targetRangeX, targetRangeY, targetRangeOmega};
                }
            }
        }
        return new double[]{0, 0, 0};
    }


    public Transform3d targetTransform(int targetNumber){
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

    public PhotonTrackedTarget returnTag (int targetNumber){
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
