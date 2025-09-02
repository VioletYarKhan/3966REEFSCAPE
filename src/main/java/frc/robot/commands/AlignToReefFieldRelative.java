// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GryphonLib.PositionCalculations;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorIO;

public class AlignToReefFieldRelative extends SequentialCommandGroup {
  private int tagID = -1;
  private Command pathCommand;
  private Command pidAlign;
  private Pose2d goalPose;

  public AlignToReefFieldRelative(BooleanSupplier isLeftScore, DriveSubsystem drivebase, IntSupplier level, ElevatorIO elevator) {
    tagID = PositionCalculations.closestReefTag(drivebase::getCurrentPose);
    goalPose = PositionCalculations.getAlignmentReefPose(tagID, level.getAsInt(), isLeftScore.getAsBoolean());
    pathCommand = drivebase.PathToPose(goalPose, 0.0);//.until(()->drivebase.getDistanceToGoal() < 0.5);
    if (level.getAsInt() == 4){
      pidAlign = PositionPIDCommand.generateCommand(drivebase, goalPose, Seconds.of(2)).until(()->elevator.atTarget(8))
      .andThen(PositionPIDCommand.generateCommand(drivebase, PositionCalculations.getFullL4Align(tagID, isLeftScore.getAsBoolean()), Seconds.of(1), false));
    } else {
      pidAlign = PositionPIDCommand.generateCommand(drivebase, goalPose, Seconds.of(2));
    }
    
    addCommands(pathCommand);
  }

  public AlignToReefFieldRelative(BooleanSupplier isLeftScore, DriveSubsystem drivebase, IntSupplier level, IntSupplier tag, ElevatorIO elevator) {
    tagID = tag.getAsInt();
    goalPose = PositionCalculations.getAlignmentReefPose(tagID, level.getAsInt(), isLeftScore.getAsBoolean());
    pathCommand = drivebase.PathToPose(goalPose, 0.0).until(()->drivebase.getDistanceToGoal() < 0.5);
    if (level.getAsInt() == 4){
      pidAlign = PositionPIDCommand.generateCommand(drivebase, goalPose, Seconds.of(2)).until(()->elevator.atTarget(8))
      .andThen(PositionPIDCommand.generateCommand(drivebase, PositionCalculations.getFullL4Align(tagID, isLeftScore.getAsBoolean()), Seconds.of(1), true));
    } else {
      pidAlign = PositionPIDCommand.generateCommand(drivebase, goalPose, Seconds.of(2));
    }
    addCommands(pathCommand, pidAlign);
  }

  public static Command generateCommand(BooleanSupplier isLeftScore, DriveSubsystem drivebase, IntSupplier level, ElevatorIO elevator){
    return new AlignToReefFieldRelative(isLeftScore, drivebase, level, elevator);
  }
}