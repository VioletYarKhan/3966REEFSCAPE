package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Wrist.WristIO;
import frc.GryphonLib.PositionCalculations;
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
                    drivetrain.AlignToTag(goalTag, level, left),
                    new WaitCommand(0.3),
                    new MoveToIntakePositions(wrist, elevator, funnel, hand).withTimeout(0.1)
                );
            } else {
                addCommands(
                    new MoveToScoringPosition(level, wrist, elevator),
                    drivetrain.PathToPose(PositionCalculations.getAlignmentReefPose(goalTag, level, left), 0.0),
                    new ParallelCommandGroup(
                        new RunCommand(()->hand.outtake(), hand).withTimeout(1),
                        new RunCommand(()->drivetrain.driveRobotRelativeChassis(new ChassisSpeeds(-0.9, 0, 0)), drivetrain).withTimeout(1)
                    ),
                    new RunCommand(()->hand.stop(), hand).withTimeout(0.1)
                );
            }
    }
}
