// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GryphonLib.PositionCalculations;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToReefFieldRelative extends SequentialCommandGroup {
  private int tagID = -1;
  private Command pathCommand;
  private Pose2d goalPose;

  public AlignToReefFieldRelative(boolean isLeftScore, DriveSubsystem drivebase) {
    addRequirements(drivebase);
    drivebase.getStates();
    tagID = PositionCalculations.closestReefTag(drivebase.getCurrentPose());
    goalPose = PositionCalculations.getAlignmentReefPose(tagID, isLeftScore);
    pathCommand = drivebase.PathToPose(goalPose);
    addCommands(pathCommand, new RunCommand(()->drivebase.driveRobotRelativeChassis(new ChassisSpeeds(0.7, 0, 0)), drivebase).withTimeout(0.5));
  }
}