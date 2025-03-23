// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.GryphonLib.PositionCalculations;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToReefFieldRelative extends Command {
  private boolean isLeftScore;
  private DriveSubsystem drivebase;
  private int tagID = -1;
  private Command pathCommand;
  private Pose2d goalPose;

  public AlignToReefFieldRelative(boolean isLeftScore, DriveSubsystem drivebase) {
    this.isLeftScore = isLeftScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
    drivebase.getStates();
  }

  @Override
  public void initialize() {
    tagID = PositionCalculations.closestReefTag(drivebase.getCurrentPose());
    goalPose = PositionCalculations.getAlignmentReefPose(tagID, isLeftScore);
    pathCommand = drivebase.PathToPose(goalPose);
    pathCommand.schedule();
  }

  @Override
  public void end(boolean interrupted) {
    pathCommand.cancel();
    new RunCommand(()->drivebase.driveRobotRelativeChassis(new ChassisSpeeds(0.4, 0, 0)), drivebase).withTimeout(0.5).schedule();
  }

  @Override
  public boolean isFinished() {
    return (PhotonUtils.getDistanceToPose(drivebase.getCurrentPose(), goalPose) < 0.1);
  }
}