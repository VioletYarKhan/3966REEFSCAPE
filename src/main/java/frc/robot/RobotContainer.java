// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import frc.GryphonLib.MovementCalculations;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final Vision m_vision = new Vision();
  public final Elevator m_elevator = new Elevator();
  public final PoseEstimatorSubsystem m_poseEst = new PoseEstimatorSubsystem(m_vision.getCamera(), m_robotDrive);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    private SendableChooser<Command> autoChooser;

    private final double[] goalFromTag = new double[]{2, 0};

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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

                double targetYaw = m_vision.targetYaw(7);
                if (m_driverController.getAButton()){
                  m_robotDrive.driveRobotRelativeChassis(new MoveTowardsTagGoal(m_vision.targetTransform(7), goalFromTag).getSpeeds());
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

  autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
