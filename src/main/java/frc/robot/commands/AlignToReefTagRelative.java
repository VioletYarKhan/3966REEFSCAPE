// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Vision;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isLeftScore;
  private Timer dontSeeTagTimer, stopTimer;
  private DriveSubsystem drivebase;
  private int tagID = -1;

  public AlignToReefTagRelative(boolean isLeftScore, DriveSubsystem drivebase) {
    xController = new PIDController(1.5, 0.0, 0);  // Vertical movement
    yController = new PIDController(1.5, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(2, 0, 0);  // Rotation
    this.isLeftScore = isLeftScore;
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

    rotController.setSetpoint(isLeftScore ? AlignmentConstants.ROT_SETPOINT_REEF_ALIGNMENT + AlignmentConstants.ROT_SETPOINT_REEF_ALIGNMENT_OFFSET : AlignmentConstants.ROT_SETPOINT_REEF_ALIGNMENT - AlignmentConstants.ROT_SETPOINT_REEF_ALIGNMENT_OFFSET);
    rotController.setTolerance(AlignmentConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(AlignmentConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(AlignmentConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isLeftScore ? AlignmentConstants.LEFT_SETPOINT_REEF_ALIGNMENT : AlignmentConstants.RIGHT_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(AlignmentConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = Vision.getBestTag();
  }

  @Override
  public void execute() {
    if (Vision.resultHasTargets()) {
      this.dontSeeTagTimer.reset();
      Transform3d cameraToTag = Vision.targetTransform(tagID);
      Transform3d robotToTag = cameraToTag.plus(VisionConstants.kCamToRobot);
      SmartDashboard.putNumber("x", robotToTag.getX());

      double xSpeed = -xController.calculate(robotToTag.getX());
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = -yController.calculate(robotToTag.getY());
      double rotValue = rotController.calculate(robotToTag.getRotation().getAngle());

      drivebase.driveRobotRelativeChassis(new ChassisSpeeds(xSpeed, ySpeed, rotValue));

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
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(AlignmentConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(AlignmentConstants.POSE_VALIDATION_TIME);
  }
}