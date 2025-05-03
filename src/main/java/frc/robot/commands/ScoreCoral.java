package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Wrist.WristIO;
import frc.GryphonLib.PositionCalculations;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.Drive.DriveSubsystem;
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
                    new MoveToIntakePositions(wrist, elevator, funnel, hand).withTimeout(0.01)
                );
            } else {
                addCommands(
                    new MoveToScoringPosition(level, wrist, elevator),
                    drivetrain.PathToPose(PositionCalculations.getAlignmentReefPose(goalTag, level, left), 0.0),
                    new RunCommand(()->hand.outtake(), hand).withTimeout(0.3),
                    new InstantCommand(()->hand.stop(), hand)
                );
            }
    }
}
