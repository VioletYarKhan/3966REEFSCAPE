package frc.GryphonLib;

import static frc.robot.Constants.VisionConstants.kRobotToCam;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class PositionCalculations {
    public static ShuffleboardTab tab = Shuffleboard.getTab("Ghosts");

    public static Pose2d goalPose;

    private static Field2d ghostField = new Field2d();
    


    public static void CreateGhostField(){
        tab.add("Ghost Field", ghostField).withPosition(2, 0).withSize(6, 4);
    }
    /**
     * @param photonCamera
     * @param robotPose2d
     * @param tagToGoal The transformation between the tag and goal pose
     * @param TAG_TO_CHASE
     * @return The goal pose
     */
    public static Pose2d getGoalPoseFromTag(PhotonCamera photonCamera,  Pose2d robotPose2d, Transform3d tagToGoal, int TAG_TO_CHASE) {
        var robotPose = 
            new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0.0, 
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
        var photonRes = photonCamera.getLatestResult();
        if (photonRes.hasTargets()) {
            // Find the tag we want to chase
            var targetOpt = photonRes.getTargets().stream()
                .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
                .findFirst();
            if (targetOpt.isPresent()) {
                var target = targetOpt.get();
                // This is new target data, so recalculate the goal
                
                // Transform the robot's pose to find the camera's pose
                var cameraPose = robotPose.transformBy(kRobotToCam);

                // Transform the camera's pose to the target's pose
                var camToTarget = target.getBestCameraToTarget();
                var targetPose = cameraPose.transformBy(camToTarget);
                
                // Transform the tag's pose to set our goal
                goalPose = targetPose.transformBy(tagToGoal).toPose2d();
                ghostField.setRobotPose(goalPose);
                return goalPose;
            }
        }
        return robotPose2d;
    }


    public static void addGhostPose(Pose2d robotPose2d, Transform3d Goal){
        var robotPose =
            new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0.0, 
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
        goalPose = robotPose.transformBy(Goal).toPose2d();
        ghostField.setRobotPose(goalPose);
    }
}
