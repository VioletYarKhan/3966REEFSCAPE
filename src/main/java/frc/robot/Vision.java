package frc.robot;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Vision extends SubsystemBase {
    private static final PhotonCamera camera1 = new PhotonCamera(VisionConstants.kCameraName1);
    private static final PhotonCamera camera2 = new PhotonCamera(VisionConstants.kCameraName2);

    private static PhotonPipelineResult result1;
    private static PhotonPipelineResult result2;

    private static final PhotonPoseEstimator poseEstimator1 = new PhotonPoseEstimator(
        VisionConstants.kTagLayout, PoseStrategy.LOWEST_AMBIGUITY, VisionConstants.kRobotToCam1);
    private static final PhotonPoseEstimator poseEstimator2 = new PhotonPoseEstimator(
        VisionConstants.kTagLayout, PoseStrategy.LOWEST_AMBIGUITY, VisionConstants.kRobotToCam2);

    @Override
    public void periodic() {
        var results1 = camera1.getAllUnreadResults();
        var results2 = camera2.getAllUnreadResults();

        result1 = results1.get(results1.size() - 1);
        SmartDashboard.putNumber("Camera1 Best Tag", result1.hasTargets() ? result1.getBestTarget().getFiducialId() : 0);
    
        result2 = results2.get(results2.size() - 1);
        SmartDashboard.putNumber("Camera2 Best Tag", result2.hasTargets() ? result2.getBestTarget().getFiducialId() : 0);
    }

    public static PhotonPipelineResult getResult1() {
        return result1;
    }

    public static PhotonPipelineResult getResult2() {
        return result2;
    }

    public static PhotonCamera getCamera1() {
        return camera1;
    }

    public static PhotonCamera getCamera2() {
        return camera2;
    }

    public static boolean resultHasTargets() {
        return (result1 != null && result1.hasTargets()) || (result2 != null && result2.hasTargets());
    }

    public static int[] tagsInFrame() {
        List<PhotonTrackedTarget> targets = getAllTargets();
        int[] tags = new int[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            tags[i] = targets.get(i).getFiducialId();
        }
        return tags;
    }

    public static List<PhotonTrackedTarget> getAllTargets() {
        List<PhotonTrackedTarget> allTargets = new ArrayList<>();
        if (result1 != null && result1.hasTargets()) {
            allTargets.addAll(result1.getTargets());
        }
        if (result2 != null && result2.hasTargets()) {
            allTargets.addAll(result2.getTargets());
        }
        return allTargets;
    }

    public static int getBestTag() {
        if (result1 != null && result1.hasTargets()) {
            return result1.getBestTarget().getFiducialId();
        }
        if (result2 != null && result2.hasTargets()) {
            return result2.getBestTarget().getFiducialId();
        }
        return 0;
    }

    public static Optional<EstimatedRobotPose> getEstimatedGlobalPoseCam1(Pose2d prevEstimatedRobotPose) {
        poseEstimator1.setReferencePose(prevEstimatedRobotPose);
        var update = poseEstimator1.update(result1);

        return update;
    }

    public static Optional<EstimatedRobotPose> getEstimatedGlobalPoseCam2(Pose2d prevEstimatedRobotPose) {
        poseEstimator2.setReferencePose(prevEstimatedRobotPose);
        var update = poseEstimator2.update(result2);

        return update;
    }

    public static double targetYaw(int targetNumber) {
        for (PhotonTrackedTarget target : getAllTargets()) {
            if (target.getFiducialId() == targetNumber) {
                return target.getYaw();
            }
        }
        return 0;
    }

    public static Transform3d targetTransform(int targetNumber) {
        for (PhotonTrackedTarget target : getAllTargets()) {
            if (target.getFiducialId() == targetNumber) {
                return target.getBestCameraToTarget();
            }
        }
        return new Transform3d();
    }

    public static PhotonTrackedTarget returnTag(int targetNumber) {
        for (PhotonTrackedTarget target : getAllTargets()) {
            if (target.getFiducialId() == targetNumber) {
                return target;
            }
        }
        return new PhotonTrackedTarget(); // Empty target
    }
}
