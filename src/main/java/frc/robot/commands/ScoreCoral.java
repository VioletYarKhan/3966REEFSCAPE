package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EffectorWrist;

public class ScoreCoral extends SequentialCommandGroup{
    public ScoreCoral(
        int level,
        boolean left,
        CoralEffector hand,
        EffectorWrist wrist,
        Elevator elevator,
        CoralFunnel funnel,
        DriveSubsystem drivetrain){

            if (level == 4) {
                addCommands(
                    new MoveToScoringPosition(level, wrist, elevator),
                    new MoveCoralToL4Position(level, hand),
                    new AlignToReefTagRelative(left, drivetrain, level)
                );
            } else {
                addCommands(
                    new MoveToScoringPosition(level, wrist, elevator),
                    new AlignToReefTagRelative(left, drivetrain, level),
                    new RunCommand(()->hand.outtake(), hand).withTimeout(1),
                    new RunCommand(()->hand.stop(), hand).withTimeout(0.1),
                    new RotateWristToLevel(4, wrist)
                );
            }
    }
}
