package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EffectorWrist;

public class ScoreCoral extends SequentialCommandGroup{
    public ScoreCoral(
        int level,
        boolean right,
        CoralEffector hand,
        EffectorWrist wrist,
        Elevator elevator,
        CoralFunnel funnel,
        DriveSubsystem drivetrain){

            if (level == 4) {
                addCommands(
                    new AlignToReefTagRelative(right, drivetrain),
                    new MoveToScoringPosition(level, wrist, elevator),
                    new MoveCoralToL4Position(level, hand),
                    // TODO: move fwd a bit
                    new MoveElevatorToLevel(0, elevator)
                );
            } else {
                addCommands(
                    new AlignToReefTagRelative(right, drivetrain),
                    new MoveToScoringPosition(level, wrist, elevator),
                    new InstantCommand(hand::outtake).withTimeout(1),
                    new InstantCommand(hand::stop),
                    new MoveElevatorToLevel(0, elevator)
                );
            }
    }
}
