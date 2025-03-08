// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import frc.GryphonLib.PositionCalculations;


import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FunnelConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.MoveCoalToL4Position;
import frc.robot.commands.MoveToIntakePositions;
import frc.robot.commands.MoveToScoringPosition;
import frc.robot.commands.MoveTowardsTagGoal;
import frc.robot.commands.RotateFunnel;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.EffectorWrist;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final Climber m_climber = new Climber();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
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

                m_robotDrive.drive(
                -MathUtil.applyDeadband(forward, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(strafe, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(turn, OIConstants.kDriveDeadband), true);
            },
          m_robotDrive));
    
    m_elevator.setDefaultCommand(
      new RunCommand(
        ()-> {  
          if (m_elevator.getPosition() > 175) {
            m_elevator.setPosition(175);
          }
        }, m_elevator)
    );

    m_coralHand.setDefaultCommand(
      new RunCommand(
        ()-> {
          if(m_wrist.getVelocity() > 900){
              m_coralHand.intake();
            } else {
              if(m_coralHand.getControlType() != ControlType.kPosition){
                m_coralHand.stop();
              }
            }
        }, m_coralHand)
    );

    m_wrist.setDefaultCommand(new RunCommand(()-> {
      
    }, m_wrist));


    m_funnel.setDefaultCommand(
      new RunCommand(
        ()-> {
          SmartDashboard.putNumber("Funnel Position", m_funnel.getPosition());
        }, m_funnel)
    );

    m_climber.setDefaultCommand(
      new RunCommand(
        ()-> {
          m_climber.stop();
        }, m_climber)
    );


  autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureButtonBindings() {
    // m_driverController.leftBumper().onTrue(new MoveTowardsTagGoal(Vision.targetTransform(Vision.getBestTag()), new double[]{0.05, 0.05, 0.05}, m_robotDrive, VisionConstants.middleReef, false).andThen(new MoveTowardsTagGoal(Vision.targetTransform(Vision.getBestTag()), new double[]{0.05, 0.05, 0.05}, m_robotDrive, VisionConstants.leftBranch, false)));
    // m_driverController.rightBumper().onTrue(new MoveTowardsTagGoal(Vision.targetTransform(Vision.getBestTag()), new double[]{0.05, 0.05, 0.05}, m_robotDrive, VisionConstants.middleReef, false).andThen(new MoveTowardsTagGoal(Vision.targetTransform(Vision.getBestTag()), new double[]{0.05, 0.05, 0.05}, m_robotDrive, VisionConstants.rightBranch, false)));
    m_driverController.a().onTrue(new MoveToScoringPosition(1, m_wrist, m_elevator, m_coralHand));
    m_driverController.x().onTrue(new MoveToScoringPosition(2, m_wrist, m_elevator, m_coralHand));
    m_driverController.b().onTrue(new MoveToScoringPosition(3, m_wrist, m_elevator, m_coralHand));
    m_driverController.y().onTrue(new MoveToScoringPosition(4, m_wrist, m_elevator, m_coralHand));
    m_driverController.rightTrigger().onTrue(new SequentialCommandGroup(new InstantCommand(() -> {
      m_coralHand.intake();
      SmartDashboard.putString("Moving to Intake", "True");
    }, m_coralHand), new MoveToIntakePositions(m_wrist, m_elevator, m_funnel)));
    m_driverController.povUp().onTrue(new InstantCommand(m_climber::climbCCW));
    m_driverController.povDown().onTrue(new InstantCommand(m_climber::climbCW));
    m_driverController.povRight().onTrue(new RotateFunnel(m_funnel, FunnelConstants.IntakeAngle));
    m_driverController.povLeft().onTrue(new RotateFunnel(m_funnel, FunnelConstants.ClimbAngle));

    m_operatorController.start().onTrue(new InstantCommand(()->m_robotDrive.zeroHeading(), m_robotDrive));
    m_operatorController.povUp().whileTrue(new InstantCommand(()->m_elevator.set(0.3), m_elevator)).onFalse(new InstantCommand(()->m_elevator.setPosition(m_elevator.getPosition()), m_elevator));
    m_operatorController.povDown().whileTrue(new InstantCommand(()->m_elevator.set(-0.3), m_elevator)).onFalse(new InstantCommand(()->m_elevator.setPosition(m_elevator.getPosition()), m_elevator));
    m_operatorController.leftBumper().whileTrue(new InstantCommand(()->m_wrist.set(-0.15), m_wrist)).onFalse(new InstantCommand(()->m_wrist.setPosition(m_wrist.getPosition()), m_wrist));
    m_operatorController.rightBumper().whileTrue(new InstantCommand(()->m_wrist.set(0.15), m_wrist)).onFalse(new InstantCommand(()->m_wrist.setPosition(m_wrist.getPosition()), m_wrist));
    m_operatorController.rightTrigger().onTrue(new InstantCommand(()->m_coralHand.goToPosition(4), m_coralHand));
    m_operatorController.leftTrigger().whileTrue(new InstantCommand(()->m_coralHand.intake(), m_coralHand)).onFalse(new InstantCommand(()->m_coralHand.stop()));
    m_operatorController.povLeft().whileTrue(new InstantCommand(()->m_funnel.set(0.3), m_funnel)).onFalse(new InstantCommand(()->m_funnel.setPosition(m_funnel.getPosition()), m_funnel));
    m_operatorController.povRight().whileTrue(new InstantCommand(()->m_funnel.set(-0.3), m_funnel)).onFalse(new InstantCommand(()->m_funnel.setPosition(m_funnel.getPosition()), m_funnel));
    m_operatorController.x().onTrue(new InstantCommand(()->m_wrist.setEncoderPosition(0), m_wrist));
    m_operatorController.a().onTrue(new InstantCommand(()->m_robotDrive.stop(), m_robotDrive));
    m_operatorController.b().onTrue(new InstantCommand(() -> m_elevator.setEncoderPosition(0), m_elevator));
    m_operatorController.y().onTrue(new InstantCommand(() -> m_robotDrive.setX(), m_robotDrive));
  }
  

  public Command getAutonomousCommand() {
    return Parser.parse(SmartDashboard.getString("Auto Code", ""));
  }
}
