// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.GryphonLib.PositionCalculations;
import frc.littletonUtils.AllianceFlipUtil;

import static frc.robot.Constants.VisionConstants.kTagLayout;

import java.util.ArrayList;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FunnelConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToReefFieldRelative;
import frc.robot.commands.MoveCoralToL4Position;
import frc.robot.commands.MoveToIntakePositions;
import frc.robot.commands.MoveToScoringPosition;
import frc.robot.commands.OperatorScoreCoal;
import frc.robot.commands.RotateFunnel;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorSimSystem;
import frc.robot.subsystems.Wrist.EffectorWrist;
import frc.robot.subsystems.Wrist.WristIO;
import frc.robot.subsystems.Wrist.WristSim;
import frc.robot.subsystems.BlinkinLEDs;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.Drive.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ElevatorIO m_elevator = Robot.isReal() ? new Elevator() : new ElevatorSimSystem();
  public final Vision m_vision = new Vision();
  public final WristIO m_wrist = Robot.isReal() ? new EffectorWrist() : new WristSim();
  private final CoralEffector m_coralHand = new CoralEffector();
  private final CoralFunnel m_funnel = new CoralFunnel();
  private final BlinkinLEDs m_lights = new BlinkinLEDs();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(1);

  Trigger handHasCoral = new Trigger(m_coralHand::hasCoral);
  int[] reefTags;
  int[] stationTags;
  
  private Pose2d operatorStationTagPose;

  int currentLevel = 0;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    reefTags = DriverStation.getAlliance().get() == Alliance.Blue ? AlignmentConstants.BLUE_REEF : AlignmentConstants.RED_REEF;
    stationTags = DriverStation.getAlliance().get() == Alliance.Blue ? AlignmentConstants.BLUE_HUMAN : AlignmentConstants.RED_HUMAN;
    operatorScoring();
    configureButtonBindings();
    handHasCoral.onTrue(new InstantCommand(m_lights::setHasCoral, m_lights)).onFalse(new InstantCommand(m_lights::setReadyIntake, m_lights));
    m_elevator.returnLigament().get().append(m_wrist.returnLigament().get());

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
         new RunCommand(
            () -> { 
              SmartDashboard.putNumber("Current Level", currentLevel);
                double forward = m_driverController.getLeftY();
                double strafe = m_driverController.getLeftX();
                double turn = m_driverController.getRightX();

                m_robotDrive.drive(
                -MathUtil.applyDeadband(forward, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(strafe, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(turn, OIConstants.kDriveDeadband), true);
            },
          m_robotDrive));
    
    m_elevator.returnSubsystem().setDefaultCommand(
      new RunCommand(
        ()-> {  
          if (m_elevator.getPosition() > 24) {
            m_elevator.setPosition(22);
          }
        }, m_elevator.returnSubsystem())
    );

    m_coralHand.setDefaultCommand(
      new RunCommand(
        ()-> {
            if(m_wrist.getVelocity() > 600){
              m_coralHand.intake();
            } else {
              if(m_coralHand.getControlType() != ControlType.kPosition){
                m_coralHand.stop();
              }
            }
        }, m_coralHand)
    );

    SmartDashboard.putNumber("Left Reef Align", AlignmentConstants.leftReefFieldAlignment);
    SmartDashboard.putNumber("Right Reef Align", AlignmentConstants.rightReefFieldAlignment);
  }

  private void configureButtonBindings() {
    m_driverController.a().onTrue(new InstantCommand(()->currentLevel = 1).andThen(new MoveToScoringPosition(1, m_wrist, m_elevator)));
    m_driverController.x().onTrue(new InstantCommand(()->currentLevel = 2).andThen(new MoveToScoringPosition(2, m_wrist, m_elevator)));
    m_driverController.b().onTrue(new InstantCommand(()->currentLevel = 3).andThen(new MoveToScoringPosition(3, m_wrist, m_elevator)));
    m_driverController.y().onTrue(new InstantCommand(()->currentLevel = 4).andThen(new MoveToScoringPosition(4, m_wrist, m_elevator).andThen(Robot.isReal() ? new MoveCoralToL4Position(4, m_coralHand) : new InstantCommand())));
    m_driverController.leftBumper().whileTrue(new RunCommand(()->new AlignToReefFieldRelative(true, m_robotDrive, ()->currentLevel).schedule(), m_robotDrive)).onFalse(new InstantCommand(m_robotDrive::stop, m_robotDrive));
    m_driverController.rightBumper().whileTrue(new RunCommand(()->new AlignToReefFieldRelative(false, m_robotDrive, ()->currentLevel).schedule(), m_robotDrive)).onFalse(new InstantCommand(m_robotDrive::stop, m_robotDrive));
    m_driverController.leftTrigger().whileTrue(new RunCommand(()->m_coralHand.intake(), m_coralHand)).onTrue(new InstantCommand(()->currentLevel = 0).andThen(new MoveToIntakePositions(m_wrist, m_elevator, m_funnel, m_coralHand)));
    m_driverController.rightTrigger().whileTrue(new RunCommand(()->m_coralHand.outtake(), m_coralHand));
    m_driverController.povRight().onTrue(new RotateFunnel(m_funnel, FunnelConstants.IntakeAngle));
    m_driverController.povLeft().onTrue(new RotateFunnel(m_funnel, FunnelConstants.ClimbAngle));

    m_operatorController.start().onTrue(new InstantCommand(()->m_robotDrive.zeroHeading(), m_robotDrive));
    m_operatorController.povUp().whileTrue(new InstantCommand(()->m_elevator.set(0.15), m_elevator.returnSubsystem())).onFalse(new InstantCommand(()->m_elevator.setPosition(m_elevator.getPosition()), m_elevator.returnSubsystem()));
    m_operatorController.povDown().whileTrue(new InstantCommand(()->m_elevator.set(-0.05), m_elevator.returnSubsystem())).onFalse(new InstantCommand(()->m_elevator.setPosition(m_elevator.getPosition()), m_elevator.returnSubsystem()));
    m_operatorController.leftBumper().whileTrue(new InstantCommand(()->m_wrist.set(-0.15), m_wrist.returnSubsystem())).onFalse(new InstantCommand(()->m_wrist.setPosition(m_wrist.getPosition()), m_wrist.returnSubsystem()));
    m_operatorController.rightBumper().whileTrue(new InstantCommand(()->m_wrist.set(0.15), m_wrist.returnSubsystem())).onFalse(new InstantCommand(()->m_wrist.setPosition(m_wrist.getPosition()), m_wrist.returnSubsystem()));
    m_operatorController.rightTrigger().onTrue(new MoveCoralToL4Position(4, m_coralHand));
    m_operatorController.leftTrigger().whileTrue(new InstantCommand(()->m_coralHand.intake(), m_coralHand)).onFalse(new InstantCommand(()->m_coralHand.stop()));
    m_operatorController.povLeft().whileTrue(new InstantCommand(()->m_funnel.set(0.5), m_funnel)).onFalse(new InstantCommand(()->m_funnel.setPosition(m_funnel.getPosition()), m_funnel));
    m_operatorController.povRight().whileTrue(new InstantCommand(()->m_funnel.set(-0.5), m_funnel)).onFalse(new InstantCommand(()->m_funnel.setPosition(m_funnel.getPosition()), m_funnel));
    m_operatorController.x().onTrue(new InstantCommand(()->m_wrist.setEncoderPosition(0), m_wrist.returnSubsystem()));
    m_operatorController.a().onTrue(new InstantCommand(()->m_robotDrive.stop(), m_robotDrive));
    m_operatorController.b().onTrue(new InstantCommand(() -> m_elevator.setEncoderPosition(0), m_elevator.returnSubsystem()));
    m_operatorController.y().onTrue(new InstantCommand(() -> m_robotDrive.setX(), m_robotDrive));
  }

  private void operatorScoring(){
    SendableChooser<Integer> operatorScoringLevel = new SendableChooser<>();
    for (int i = 2; i <= 3; i++){
      operatorScoringLevel.addOption(""+i, i);
    }
    operatorScoringLevel.setDefaultOption("4", 4);
    SmartDashboard.putData("Operator Height Chooser", operatorScoringLevel);
    for(int i = 1; i <= 6; i++){
      int side = i;
      for (Boolean left : new Boolean[]{true, false}){
        SmartDashboard.putData("Operator Controls/" + i + (left ? "L" : "R"), 
        new InstantCommand(()->
          new SequentialCommandGroup(
          m_robotDrive.AlignToTagFar(reefTags[side - 1]),
          new OperatorScoreCoal(
            left,
            m_coralHand,
            m_wrist,
            m_elevator,
            m_funnel,
            m_robotDrive,
            reefTags[side - 1]
          )
        ).schedule()));
      }
    }

    // Workaround of weird Java thing
    int[] station = new int[]{0};
    for (int i = 1; i <=2; i++){
      station[0] = i;
      operatorStationTagPose = kTagLayout.getTagPose(stationTags[station[0] - 1]).get().toPose2d();
      SmartDashboard.putData(("Operator Controls/" + i + "C"),
        new ParallelCommandGroup(
          new MoveToIntakePositions(m_wrist, m_elevator, m_funnel, m_coralHand),
          m_robotDrive.PathToPose(PositionCalculations.translateCoordinates(operatorStationTagPose, operatorStationTagPose.getRotation().getDegrees(), 0.3), 0.0),
          new SequentialCommandGroup(
            (new RunCommand(() -> m_coralHand.intake(), m_coralHand).until(()->m_coralHand.hasCoral())),
            new RunCommand(() -> m_coralHand.intake(), m_coralHand).withTimeout(0.3))
        )
      );
    }
  }
  
  public Command parseAutoCommand(){
    try {
      SequentialCommandGroup autoRoutine = new SequentialCommandGroup();
    
    int[] reefTags = DriverStation.getAlliance().get() == Alliance.Blue ? AlignmentConstants.BLUE_REEF : AlignmentConstants.RED_REEF;
    int[] stationTags = DriverStation.getAlliance().get() == Alliance.Blue ? AlignmentConstants.BLUE_HUMAN : AlignmentConstants.RED_HUMAN;

    String autoString = SmartDashboard.getString("Auto Code", "1S-13L-1C-63L-1C-63R");
    
    ArrayList<Command> commands = Parser.parse(autoString);
    Parser.SetPositionCommand setPositionCommand;
    try {
      setPositionCommand = (Parser.SetPositionCommand) commands.get(0);
    } catch (IndexOutOfBoundsException e){
      return new SequentialCommandGroup();
    }
    
    autoRoutine.addCommands(
      new InstantCommand(()->m_robotDrive.setHeading(180), m_robotDrive),
      new InstantCommand(()->m_robotDrive.setCurrentPose(
        AllianceFlipUtil.apply(AutoConstants.startPositions[setPositionCommand.getPosition()])), m_robotDrive)
    );
    commands.remove(0);
    // Default is placeholder
    ArrayList<Command> convertedCommands = new ArrayList<>();
    for (int i = 0; i < commands.size(); i ++) {
      Command fullCommand = commands.get(i);

      if (fullCommand instanceof Parser.PutCoralCommand) {
        Parser.PutCoralCommand putCmd = (Parser.PutCoralCommand) fullCommand;
        // Command pathCommand = m_robotDrive.AlignToTagFar(reefTags[putCmd.getSide() - 1]);
        Command pathCommand = m_robotDrive.AlignToTag(reefTags[putCmd.getSide() - 1], putCmd.getLevel(), putCmd.getLeft()).until(m_robotDrive::closeToGoal);
          
          // Build the scoring sequence that will run concurrently with the path command.
          Command subsystemMovement = new SequentialCommandGroup(
              new WaitUntilCommand(()->(m_robotDrive.getDistanceToGoal() < 2.5)),
              new MoveToScoringPosition(putCmd.getLevel(), m_wrist, m_elevator)
          );
          
          
          // Create a sequential group:
          // 1. Run the path command and scoringSequence in parallel.
          // 2. Then run ScoreCoral.
          Command fullSequence = new SequentialCommandGroup(
              new ParallelCommandGroup(pathCommand, new SequentialCommandGroup(
                new SequentialCommandGroup(
                  (Robot.isReal() ? new RunCommand(() -> m_coralHand.intake(), m_coralHand).until(()->m_coralHand.hasCoral()) : new InstantCommand()),
                  new RunCommand(() -> m_coralHand.intake(), m_coralHand).withTimeout(0.3)), subsystemMovement)),
              new ScoreCoral(
                  putCmd.getLevel(),
                  putCmd.getLeft(),
                  m_coralHand,
                  m_wrist,
                  m_elevator,
                  m_funnel,
                  m_robotDrive,
                  reefTags[putCmd.getSide() - 1]
              )
          );
          convertedCommands.add(fullSequence);
        } else if (fullCommand instanceof Parser.GetCoralCommand) {
          Parser.GetCoralCommand getCmd = (Parser.GetCoralCommand) fullCommand;
          Pose2d stationTagPose = kTagLayout.getTagPose(stationTags[getCmd.getStation() - 1]).get().toPose2d();
          Command pathCommand = m_robotDrive.PathToPose(PositionCalculations.translateCoordinates(stationTagPose, stationTagPose.getRotation().getDegrees(), 0.3), 0.0);
            // For GetCoralCommand, run your intake sequence in parallel with the path command.
            Command intakeSequence = new SequentialCommandGroup(
                new MoveToIntakePositions(m_wrist, m_elevator, m_funnel, m_coralHand),
                new RunCommand(() -> m_coralHand.intake(), m_coralHand).until(()->m_coralHand.hasCoral()).withTimeout(1)
            );

            ParallelRaceGroup fullSequence = new ParallelCommandGroup(pathCommand, intakeSequence).until(m_coralHand::hasCoral);

            convertedCommands.add(fullSequence);
        }
    }
    for (Command command : convertedCommands){
      autoRoutine.addCommands(command);
    }
    return autoRoutine.withTimeout(15);
    } catch (Exception e){
      return new SequentialCommandGroup();
    }
  }
}
