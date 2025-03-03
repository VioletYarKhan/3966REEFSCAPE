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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.EffectorWrist;
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
  public final Vision m_vision = new Vision();
  public final EffectorWrist m_wrist = new EffectorWrist();
  private final CoralEffector m_coralHand = new CoralEffector();
  private final CoralFunnel m_Funnel = new CoralFunnel();
  private final Climber m_Climber = new Climber();

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
                if(m_driverController.getLeftBumperButton()){
                  Pose2d goalPose = PositionCalculations.getGoalPoseFromTag(Vision.getCamera(), m_robotDrive.getCurrentPose(), AutoConstants.leftBranchCoral, Vision.getBestTag());
                } else if (m_driverController.getRightBumperButton()){
                  Pose2d goalPose = PositionCalculations.getGoalPoseFromTag(Vision.getCamera(), m_robotDrive.getCurrentPose(), AutoConstants.rightBranchCoral, Vision.getBestTag());
                }
                if (m_driverController.getAButton()){
                  // m_robotDrive.PathToPose(goalPose);
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
          if (m_elevator.getPosition() > 180){
            m_elevator.set(-0.2);
          } else if (m_driverController.getXButton()){
            m_elevator.setPosition(ElevatorConstants.L3Height);
          } else if (m_driverController.getBButton()){
            // m_elevator.setPosition(40);
          }else if (m_driverController.getYButton()){
            // m_elevator.setPosition(175);
          } else {
            // m_elevator.setPosition(m_elevator.getPosition());
            SmartDashboard.putString("Elevator", "stop");
          }
        }, m_elevator)
    );
    
    m_wrist.setDefaultCommand(
      new RunCommand(
        ()-> {
          SmartDashboard.putNumber("Wrist Position", m_wrist.getPosition());
          SmartDashboard.putNumber("Wrist Speed", m_wrist.getVelocity());
          if (m_driverController.getLeftTriggerAxis() > 0.5){
            m_wrist.setPosition(WristConstants.IntakeAngle);
          } else if (m_driverController.getAButtonPressed()){
            m_wrist.setPosition(WristConstants.L1Angle);
          } else if (m_driverController.getBButtonPressed()){
            m_wrist.setPosition(WristConstants.L2_3Angle);
          } else if (m_driverController.getYButtonPressed()){
            m_wrist.setPosition(WristConstants.L4Angle);
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
          } else if (m_wrist.getPosition() > WristConstants.L2_3Angle) {
            m_coralHand.intake();
          }
        }, m_coralHand)
    );

    m_Funnel.setDefaultCommand(
      new RunCommand(
        ()-> {
          if (m_driverController.getLeftBumperButton()){
            m_Funnel.set(0.3);
          } else if (m_driverController.getRightBumperButton()){
            m_Funnel.set(-0.3);
          } else{
            // m_Funnel.set(0);
          }
        }, m_Funnel)
    );

  autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }



  private void configureButtonBindings() {
    m_operatorController.leftBumper().whileTrue(new InstantCommand(()->m_coralHand.intake(), m_coralHand)).onFalse(new InstantCommand(()->m_coralHand.stop(), m_coralHand));
    m_operatorController.rightBumper().whileTrue(new InstantCommand(()->m_coralHand.outtake(), m_coralHand)).onFalse(new InstantCommand(()->m_coralHand.stop(), m_coralHand));
    m_operatorController.povUp().whileTrue(new InstantCommand(()->m_elevator.set(0.3), m_elevator)).onFalse(new InstantCommand(()->m_elevator.setPosition(m_elevator.getPosition()), m_elevator));
    m_operatorController.povDown().whileTrue(new InstantCommand(()->m_elevator.set(-0.3), m_elevator)).onFalse(new InstantCommand(()->m_elevator.setPosition(m_elevator.getPosition()), m_elevator));
    m_operatorController.povRight().whileTrue(new InstantCommand(()->m_wrist.set(-0.1), m_wrist)).onFalse(new InstantCommand(()->m_wrist.setPosition(m_wrist.getPosition()), m_wrist));
    m_operatorController.povLeft().whileTrue(new InstantCommand(()->m_wrist.set(0.1), m_wrist)).onFalse(new InstantCommand(()->m_wrist.setPosition(m_wrist.getPosition()), m_wrist));
    m_operatorController.a().whileTrue(new InstantCommand(()->m_Climber.climberCCW(), m_Climber)).onFalse(new InstantCommand(()->m_Climber.climberStop(), m_Climber));
    m_operatorController.b().whileTrue(new InstantCommand(()->m_Climber.climberCW(), m_Climber)).onFalse(new InstantCommand(()->m_Climber.climberStop(), m_Climber));
    m_operatorController.leftTrigger().whileTrue(new InstantCommand(()->m_Funnel.set(0.3), m_Funnel)).onFalse(new InstantCommand(()->m_Funnel.set(0), m_Funnel));  
    m_operatorController.rightTrigger().whileTrue(new InstantCommand(()->m_Funnel.set(-0.3), m_Funnel)).onFalse(new InstantCommand(()->m_Funnel.set(0), m_Funnel));  
    m_operatorController.x().whileTrue(new InstantCommand(()->m_wrist.setEncoderPosition(0), m_wrist));
    m_operatorController.start().onTrue(new InstantCommand(()->m_robotDrive.zeroHeading(), m_robotDrive));
  }

  public Command getAutonomousCommand() {
    return Parser.parse(SmartDashboard.getString("Auto Code", ""));
  }
}
