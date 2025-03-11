package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.Vision;
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
                    new WaitUntilCommand(Vision::resultHasTargets),
                    new AlignToReefTagRelative(left, drivetrain),
                    new RunCommand(()->drivetrain.driveRobotRelativeChassis(new ChassisSpeeds(0.2, 0, 0)), drivetrain).withTimeout(0.2),
                    new MoveToIntakePositions(wrist, elevator, funnel)
                );
            } else {
                addCommands(
                    new MoveToScoringPosition(level, wrist, elevator),
                    new WaitUntilCommand(Vision::resultHasTargets),
                    new AlignToReefTagRelative(left, drivetrain),
                    new RunCommand(()->hand.outtake(), hand).withTimeout(1),
                    new RunCommand(()->hand.stop(), hand).withTimeout(0.1),
                    new RotateWristToLevel(4, wrist),
                    new MoveToIntakePositions(wrist, elevator, funnel)
                );
            }
    }
}
