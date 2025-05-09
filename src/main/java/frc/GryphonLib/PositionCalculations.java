package frc.GryphonLib;

import static frc.robot.Constants.VisionConstants.kTagLayout;

import java.util.function.Supplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AlignmentConstants;

public class PositionCalculations {
    public static Pose2d goalPose;
  
    public static Pose2d translateCoordinates(
        Pose2d originalPose, double degreesRotate, double distance
    ){
        double newXCoord = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate)) * distance);
        double newYCoord = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate)) * distance);

        return new Pose2d(newXCoord, newYCoord, originalPose.getRotation());
    }


    public static Pose2d getAlignmentReefPose(int tag, int level, boolean isLeftScore){
        Transform2d transform = new Transform2d();
        if (level != 1){
        transform = new Transform2d(0.7, isLeftScore ? -SmartDashboard.getNumber("Left Reef Align", AlignmentConstants.leftReefFieldAlignment) : SmartDashboard.getNumber("Right Reef Align", AlignmentConstants.rightReefFieldAlignment), new Rotation2d());
        if (level == 4){
            transform = new Transform2d(0.4, transform.getY(), new Rotation2d());
        }
        } else {
        transform = new Transform2d(0.6, 0, new Rotation2d(isLeftScore ? Math.PI/6 : -Math.PI/6));
        }

        Pose2d tagPose = kTagLayout.getTagPose(tag).get().toPose2d();
        Pose2d goalPose = translateCoordinates(tagPose, tagPose.getRotation().getDegrees(), transform.getX());
        goalPose = translateCoordinates(goalPose, tagPose.getRotation().getDegrees() + 90, transform.getY());

        return goalPose.transformBy(new Transform2d(0, 0, new Rotation2d(Math.PI).plus(transform.getRotation())));
    }

    public static Pose2d getStraightOutPose(int tag){
        Pose2d tagPose = kTagLayout.getTagPose(tag).get().toPose2d();
        Pose2d goalPose = translateCoordinates(tagPose, tagPose.getRotation().getDegrees(), 1.5);

        return goalPose.transformBy(new Transform2d(0, 0, new Rotation2d(Math.PI)));
    }

    public static int closestReefTag(Supplier<Pose2d> currPoseSupplier){
        Pose2d currPose = currPoseSupplier.get();
        int[] reefTags = DriverStation.getAlliance().get() == Alliance.Blue ? AlignmentConstants.BLUE_REEF : AlignmentConstants.RED_REEF;
        double closestDistance = Double.MAX_VALUE;
        int closestTag = 0;
        for (int tag : reefTags){
            double distance = PhotonUtils.getDistanceToPose(currPose, kTagLayout.getTagPose(tag).get().toPose2d());
            if (distance < closestDistance){
                closestTag = tag;
                closestDistance = distance;
            }
        }
        return closestTag;
    }
}
