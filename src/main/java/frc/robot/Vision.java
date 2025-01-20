package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraName);
    private PhotonPipelineResult result;
    private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, VisionConstants.kRobotToCam);
    private int[] tags;


    public void periodic() {
        result = camera.getLatestResult();
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


    public double targetDistance(int targetNumber){
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == targetNumber) {
                    // Found Tag, record its information
                    double targetRange =
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        0.5, // Measured with a tape measure, or in CAD.
                                        1.435, // From 2024 game manual for ID 7
                                        Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                                        Units.degreesToRadians(target.getPitch()));
                    return targetRange;
                }
            }
        }
        return 0;
    }
}
