// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GryphonLib.PositionCalculations;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToReefFieldRelative extends SequentialCommandGroup {
  private int tagID = -1;
  private Command pathCommand;
  private Pose2d goalPose;

  public AlignToReefFieldRelative(boolean isLeftScore, DriveSubsystem drivebase, IntSupplier level) {
    addRequirements(drivebase);
    tagID = PositionCalculations.closestReefTag(drivebase::getCurrentPose);
    Transform2d transform = new Transform2d();
    if (level.getAsInt() != 1){
      transform = new Transform2d(0.2, isLeftScore ? -SmartDashboard.getNumber("Left Reef Align", AlignmentConstants.leftReefFieldAlignment) : SmartDashboard.getNumber("Right Reef Align", AlignmentConstants.rightReefFieldAlignment), new Rotation2d());
      if (level.getAsInt() == 4){
        transform = new Transform2d(0.1, transform.getY(), new Rotation2d());
      }
    } else {
      transform = new Transform2d(0.2, 0, new Rotation2d(isLeftScore ? Math.PI/6 : -Math.PI/6));
    }
    goalPose = PositionCalculations.getAlignmentReefPose(tagID, transform);
    pathCommand = drivebase.PathToPose(goalPose, 0.0);
    addCommands(pathCommand, new RunCommand(()->drivebase.driveRobotRelativeChassis(new ChassisSpeeds(0.7, 0, 0)), drivebase).withTimeout(0.5));
  }
}