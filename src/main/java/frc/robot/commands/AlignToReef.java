// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import java.util.List;
import java.util.function.IntSupplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.GryphonLib.PositionCalculations;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToReef extends SequentialCommandGroup {
  private int tagID = -1;
  private Command pathCommand;
  private Pose2d goalPose;
  private PIDController forwardPidController = new PIDController(1, 0, 0);
  private PIDController strafePidController = new PIDController(1, 0, 0);
  private ProfiledPIDController thetaPidController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(2*Math.PI, Math.PI));
  private HolonomicDriveController hDriveController = new HolonomicDriveController(forwardPidController, strafePidController, thetaPidController);

  TrajectoryConfig alignConfig = new TrajectoryConfig(1, 1).setKinematics(DriveConstants.kDriveKinematics);

  public AlignToReef(boolean isLeftScore, DriveSubsystem drivebase, IntSupplier level) {
    addRequirements(drivebase);
    tagID = PositionCalculations.closestReefTag(drivebase::getCurrentPose);
    goalPose = PositionCalculations.getAlignmentReefPose(tagID, level.getAsInt(), isLeftScore);
    drivebase.getField2d().getObject("Goal Pose").setPose(goalPose);

    pathCommand = drivebase.PathToPose(goalPose, 0.0).until(()->(drivebase.getDistanceToGoal() < 0.5));
    
    
    Trajectory alignTrajectory = TrajectoryGenerator.generateTrajectory(
        drivebase.getCurrentPose(),
        List.of(),
        goalPose,
        alignConfig);

    SwerveControllerCommand closeAlign = new SwerveControllerCommand(
      alignTrajectory,
      drivebase::getCurrentPose,
      DriveConstants.kDriveKinematics,
      hDriveController,
      () -> goalPose.getRotation(),
      drivebase::setModuleStates,
      drivebase);
    
    addCommands(pathCommand, closeAlign);
  }
}