package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Wrist.WristIO;
import frc.GryphonLib.PositionCalculations;
import frc.robot.Robot;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorIO;

public class ScoreCoral extends SequentialCommandGroup{
    public ScoreCoral(
        int level,
        boolean left,
        CoralEffector hand,
        WristIO wrist,
        ElevatorIO elevator,
        CoralFunnel funnel,
        DriveSubsystem drivetrain,
        int goalTag){

            if (level == 4) {
                addCommands(
                    new MoveToScoringPosition(level, wrist, elevator),
                    new MoveCoralToL4Position(level, hand),
                    PositionPIDCommand.generateCommand(drivetrain, PositionCalculations.getAlignmentReefPose(goalTag, level, left), Seconds.of(1.5)),
                    new RunCommand(()->{if (Robot.isReal()) {drivetrain.driveRobotRelativeChassis(new ChassisSpeeds(0.1, 0, 0));} else{drivetrain.stop();}}, drivetrain).withTimeout(0.2),
                    new WaitCommand(0.3),
                    new MoveToIntakePositions(wrist, elevator, funnel, hand).withTimeout(0.5)
                );
            } else {
                addCommands(
                    new MoveToScoringPosition(level, wrist, elevator),
                    drivetrain.PathToPose(PositionCalculations.getAlignmentReefPose(goalTag, level, left), 0.0),
                    new ParallelCommandGroup(
                        new RunCommand(()->hand.outtake(()->level), hand).withTimeout(1),
                        new RunCommand(()->drivetrain.driveRobotRelativeChassis(new ChassisSpeeds(-0.2, 0, 0)), drivetrain).withTimeout(0.5)
                    ),
                    new RunCommand(()->hand.stop(), hand).withTimeout(0.1)
                );
            }
    }
}
