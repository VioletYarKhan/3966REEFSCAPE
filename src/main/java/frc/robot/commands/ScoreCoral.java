package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Wrist.WristIO;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorIO;

public class ScoreCoral extends SequentialCommandGroup{
    public ScoreCoral(
        IntSupplier level,
        BooleanSupplier left,
        CoralEffector hand,
        WristIO wrist,
        ElevatorIO elevator,
        CoralFunnel funnel,
        DriveSubsystem drivetrain,
        IntSupplier goalTag
        ){

            if (level.getAsInt() == 4) {
                addCommands(
                    new MoveToScoringPosition(level.getAsInt(), wrist, elevator),
                    new MoveCoralToL4Position(level.getAsInt(), hand),
                    new AlignToReefFieldRelative(left, drivetrain, level, goalTag, elevator),
                    new WaitCommand(0.1),
                    new MoveToIntakePositions(wrist, elevator, funnel, hand).withTimeout(0.2)
                );
            } else {
                addCommands(
                    new MoveToScoringPosition(level.getAsInt(), wrist, elevator),
                    new AlignToReefFieldRelative(left, drivetrain, level, elevator),
                    new ParallelCommandGroup(
                        new RunCommand(()->hand.outtake(), hand).withTimeout(1),
                        new RunCommand(()->drivetrain.driveRobotRelativeChassis(new ChassisSpeeds(-0.9, 0, 0)), drivetrain).withTimeout(1)
                    ),
                    new MoveToIntakePositions(wrist, elevator, funnel, hand).withTimeout(0.1),
                    new RunCommand(()->hand.stop(), hand).withTimeout(0.1)
                );
            }
    }
}
