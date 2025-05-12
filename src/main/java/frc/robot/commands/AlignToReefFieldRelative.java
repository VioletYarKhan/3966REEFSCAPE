// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import static edu.wpi.first.units.Units.Seconds;

import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GryphonLib.PositionCalculations;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToReefFieldRelative extends SequentialCommandGroup {
  private int tagID = -1;
  private Command pathCommand;
  private Pose2d goalPose;

  public AlignToReefFieldRelative(boolean isLeftScore, DriveSubsystem drivebase, IntSupplier level) {
    addRequirements(drivebase);
    tagID = PositionCalculations.closestReefTag(drivebase::getCurrentPose);
    goalPose = PositionCalculations.getAlignmentReefPose(tagID, level.getAsInt(), isLeftScore);
    pathCommand = drivebase.PathToPose(goalPose, 0.0).until(()->drivebase.getDistanceToGoal() < 0.5);
    addCommands(pathCommand, PositionPIDCommand.generateCommand(drivebase, goalPose, Seconds.of(2)));
  }
}