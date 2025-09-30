package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GryphonLib.PositionCalculations;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Wrist.WristIO;

public class AlgaeRemoval extends SequentialCommandGroup {

    private static final Set<Integer> L3_TAGS = Set.of(7, 9, 11, 18, 20, 22);
    
    public AlgaeRemoval(
        CoralEffector hand,
        WristIO wrist,
        ElevatorIO elevator,
        DriveSubsystem drivetrain,
        CoralFunnel funnel
    ){
        int goalTag = PositionCalculations.closestReefTag(drivetrain::getCurrentPose);
        addCommands(
            new AlignToReefFieldRelative(false, drivetrain, () -> 0),
            new ParallelCommandGroup(
                new InstantCommand(() -> hand.outtake(() -> 2)),
                new MoveToScoringPosition(getLevel(goalTag), wrist, elevator)
            ).withTimeout(1),
            new ParallelCommandGroup(
                new InstantCommand(hand::stop),
                new MoveToIntakePositions(wrist, elevator, funnel, hand)
            )
        );
    }

    private int getLevel(int goalTag) {
        return L3_TAGS.contains(goalTag) ? 3 : 2;
    }
}
