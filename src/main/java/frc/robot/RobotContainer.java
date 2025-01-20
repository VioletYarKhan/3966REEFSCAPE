// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  // public final Elevator m_elevator = new Elevator();
  public final PoseEstimatorSubsystem m_poseEst = new PoseEstimatorSubsystem(m_vision.getCamera(), m_robotDrive);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);


    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    public Field2d m_field = new Field2d();

  // Commands
    private Supplier<Integer> tagToChase;
    private final MoveToTag ChaseTagCommand = new MoveToTag(m_vision.getResult(), m_robotDrive, ()->m_field.getRobotPose(), tagToChase);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
         new RunCommand(
            () -> {
                m_field.setRobotPose(m_poseEst.getCurrentPose());
                if (m_driverController.getRightBumperButton()){
                  tagToChase = ()->m_vision.getBestTag();
                  ChaseTagCommand.schedule();
                } else{
                  double forward = m_driverController.getLeftY();
                  double strafe = m_driverController.getLeftX();
                  double turn = m_driverController.getRightX();

                  if (m_driverController.getStartButton()){
                    turn = Math.min(Math.abs(m_vision.targetYaw(7))/20, 1);
                  }

                  m_robotDrive.drive(
                  -MathUtil.applyDeadband(forward, OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(strafe, OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(turn, OIConstants.kDriveDeadband), true);}},
            m_robotDrive));

    /*m_elevator.setDefaultCommand(
      new RunCommand(
        () -> {
          if (m_driverController.getPOV() == 0) {
            m_elevator.set(0.2);
          } else if (m_driverController.getPOV() == 180) {
            m_elevator.set(-0.15);
          } else {
            if (m_driverController.getYButton()){
              m_elevator.setPosition(6);
            } else if (m_driverController.getAButton()){
              m_elevator.setPosition(0);
            }
          }
        }
      , m_elevator));*/

  autoChooser.setDefaultOption("None", new RunCommand((()-> m_robotDrive.setX()), m_robotDrive));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
