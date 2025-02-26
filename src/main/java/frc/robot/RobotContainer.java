// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import frc.GryphonLib.MovementCalculations;
import frc.GryphonLib.PositionCalculations;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.coralEffector;
import frc.robot.subsystems.effectorWrist;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final Elevator m_elevator = new Elevator();
  public final effectorWrist m_wrist = new effectorWrist();
  private final coralEffector m_coralHand = new coralEffector();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(1);

    private SendableChooser<Command> autoChooser;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    PositionCalculations.CreateGhostField();
    configureButtonBindings();
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
         new RunCommand(
            () -> { 
                double forward = m_driverController.getLeftY();
                double strafe = m_driverController.getLeftX();
                double turn = m_driverController.getRightX();

                double targetYaw = Vision.targetYaw(7);
                if (m_driverController.getAButton()){
                  Pose2d goalPose = PositionCalculations.getGoalPoseFromTag(Vision.getCamera(), m_robotDrive.getCurrentPose(), new Transform3d(), Vision.getBestTag());
                  m_robotDrive.PathToPose(goalPose);
                } else{
                  if (m_driverController.getStartButton()){
                    turn = MovementCalculations.getTurnRate(targetYaw, 0.5 * 1);
                  }

                    m_robotDrive.drive(
                    -MathUtil.applyDeadband(forward, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(strafe, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(turn, OIConstants.kDriveDeadband), true);}},
          m_robotDrive));
    
    m_elevator.setDefaultCommand(
      new RunCommand(
        ()-> {
          SmartDashboard.putNumber("Elevator Position", m_elevator.getPosition());
          if (m_driverController.getPOV() == 0){
            m_elevator.set(0.2);
          } else if (m_driverController.getPOV() == 180){
            m_elevator.set(-0.2);
          } else{
            m_elevator.set(0);
          }
        }, m_elevator)
    );

    m_wrist.setDefaultCommand(
      new RunCommand(
        ()-> {
          SmartDashboard.putNumber("Wrist Position", m_wrist.getPosition());
          if (m_driverController.getPOV() == 90){
            m_wrist.set(0.2);
          } else if (m_driverController.getPOV() == 270){
            m_wrist.set(-0.2);
          } else{
            m_wrist.set(0);
          }
        }, m_wrist)
    );

    m_coralHand.setDefaultCommand(
      new RunCommand(
        ()-> {
          if (m_driverController.getRightTriggerAxis() > 0.5){
            m_coralHand.outtake();
          } else if (m_driverController.getLeftTriggerAxis() > 0.5){
            m_coralHand.intake();
          } else{
            m_coralHand.stop();
          }
        }, m_coralHand)
    );

  autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureButtonBindings() {
    m_operatorController.a().onTrue(new InstantCommand(()->m_robotDrive.stop(), m_robotDrive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
