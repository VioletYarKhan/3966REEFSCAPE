// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private DriveSubsystem drivebase;
  private double tagID = -1;

  public AlignToReefTagRelative(boolean isRightScore, DriveSubsystem drivebase) {
    xController = new PIDController(1.0, 0.0, 0);  // Vertical movement
    yController = new PIDController(1.0, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(1.0, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
    drivebase.getStates();
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(AlignmentConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(AlignmentConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(AlignmentConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(AlignmentConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? AlignmentConstants.Y_SETPOINT_REEF_ALIGNMENT : -AlignmentConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(AlignmentConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID(VisionConstants.kCameraName);
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV(VisionConstants.kCameraName) && LimelightHelpers.getFiducialID(VisionConstants.kCameraName) == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace(VisionConstants.kCameraName);
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      drivebase.drive(xSpeed, ySpeed, rotValue, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(0, 0, 0, false);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(AlignmentConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(AlignmentConstants.POSE_VALIDATION_TIME);
  }
}