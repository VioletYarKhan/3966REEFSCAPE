// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import frc.GryphonLib.PositionCalculations;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FunnelConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.MoveElevatorToLevel;
import frc.robot.commands.MoveToIntakePositions;
import frc.robot.commands.MoveToScoringPosition;
import frc.robot.commands.MoveTowardsTagGoal;
import frc.robot.commands.RotateFunnel;
import frc.robot.commands.RotateWristToLevel;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
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
  private final CoralFunnel m_funnel = new CoralFunnel();

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

                if (m_driverController.getLeftBumperButton()){
                  new MoveTowardsTagGoal(Vision.targetTransform(Vision.getBestTag()), new double[]{0.05, 0.05, 0.05}, m_robotDrive, VisionConstants.middleReef, false).andThen(new MoveTowardsTagGoal(Vision.targetTransform(Vision.getBestTag()), new double[]{0.05, 0.05, 0.05}, m_robotDrive, VisionConstants.leftBranch, false)).schedule();
                } else if (m_driverController.getRightBumperButton()){
                  new MoveTowardsTagGoal(Vision.targetTransform(Vision.getBestTag()), new double[]{0.05, 0.05, 0.05}, m_robotDrive, VisionConstants.middleReef, false).andThen(new MoveTowardsTagGoal(Vision.targetTransform(Vision.getBestTag()), new double[]{0.05, 0.05, 0.05}, m_robotDrive, VisionConstants.rightBranch, false)).schedule();
                } else{
                  m_robotDrive.drive(
                  -MathUtil.applyDeadband(forward, OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(strafe, OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(turn, OIConstants.kDriveDeadband), true);}},
          m_robotDrive));
    
    m_elevator.setDefaultCommand(
      new RunCommand(
        ()-> {
          SmartDashboard.putNumber("Elevator Position", m_elevator.getPosition());
          if (m_elevator.getPosition() > 175){
            m_elevator.set(-0.2);
          } else if (m_driverController.getAButtonPressed()){
            new MoveToScoringPosition(1, m_wrist, m_elevator).schedule(); 
          } else if (m_driverController.getXButtonPressed()){
            new MoveToScoringPosition(2, m_wrist, m_elevator).schedule();
          } else if (m_driverController.getBButtonPressed()){
            new MoveToScoringPosition(3, m_wrist, m_elevator).schedule(); 
          } else if (m_driverController.getYButtonPressed()){
            new MoveToScoringPosition(4, m_wrist, m_elevator).schedule();
          }
        }, m_elevator)
    );

    m_coralHand.setDefaultCommand(
      new RunCommand(
        ()-> {
          if (m_driverController.getRightTriggerAxis() > 0.5){
            m_coralHand.outtake();
          } else if (m_driverController.getLeftTriggerAxis() > 0.5){
            new MoveToIntakePositions(m_wrist, m_elevator).schedule();
            m_coralHand.intake();
          } else{
            if(m_wrist.getPosition() > WristConstants.L4Angle){
              m_coralHand.intake();
            } else {
              m_coralHand.stop();
            }
          }
        }, m_coralHand)
    );


    m_funnel.setDefaultCommand(
      new RunCommand(
        ()-> {
          SmartDashboard.putNumber("Funnel Position", m_funnel.getPosition());
          if (m_driverController.getPOV() == 90) {
            new RotateFunnel(m_funnel, FunnelConstants.IntakeAngle).schedule();
          } else if (m_driverController.getPOV() == 270) {
            new RotateFunnel(m_funnel, FunnelConstants.ClimbAngle).schedule();
          }
        }, m_funnel)
    );


  autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureButtonBindings() {
    m_operatorController.start().onTrue(new InstantCommand(()->m_robotDrive.zeroHeading(), m_robotDrive));
    m_operatorController.povUp().whileTrue(new InstantCommand(()->m_elevator.set(0.3), m_elevator)).onFalse(new InstantCommand(()->m_elevator.setPosition(m_elevator.getPosition()), m_elevator));
    m_operatorController.povDown().whileTrue(new InstantCommand(()->m_elevator.set(-0.3), m_elevator)).onFalse(new InstantCommand(()->m_elevator.setPosition(m_elevator.getPosition()), m_elevator));
    m_operatorController.leftBumper().whileTrue(new InstantCommand(()->m_wrist.set(-0.15), m_wrist)).onFalse(new InstantCommand(()->m_wrist.setPosition(m_wrist.getPosition()), m_wrist));
    m_operatorController.rightBumper().whileTrue(new InstantCommand(()->m_wrist.set(0.15), m_wrist)).onFalse(new InstantCommand(()->m_wrist.setPosition(m_wrist.getPosition()), m_wrist));
    m_operatorController.rightTrigger().whileTrue(new InstantCommand(()->m_coralHand.outtake(), m_coralHand)).onFalse(new InstantCommand(()->m_coralHand.stop()));
    m_operatorController.leftTrigger().whileTrue(new InstantCommand(()->m_coralHand.intake(), m_coralHand)).onFalse(new InstantCommand(()->m_coralHand.stop()));
    m_operatorController.povLeft().whileTrue(new InstantCommand(()->m_funnel.set(0.3), m_funnel)).onFalse(new InstantCommand(()->m_funnel.setPosition(m_funnel.getPosition()), m_funnel));
    m_operatorController.povRight().whileTrue(new InstantCommand(()->m_funnel.set(-0.3), m_funnel)).onFalse(new InstantCommand(()->m_funnel.setPosition(m_funnel.getPosition()), m_funnel));
    m_operatorController.x().onTrue(new InstantCommand(()->m_wrist.setEncoderPosition(0), m_wrist));
    m_operatorController.a().onTrue(new InstantCommand(()->m_robotDrive.stop(), m_robotDrive));
    m_operatorController.b().onTrue(new InstantCommand(() -> m_elevator.setEncoderPosition(0)));
    m_operatorController.y().onTrue(new InstantCommand(() -> m_robotDrive.setX(), m_robotDrive));
  }
  

  public Command getAutonomousCommand() {
    return Parser.parse(SmartDashboard.getString("Auto Code", ""));
  }
}
