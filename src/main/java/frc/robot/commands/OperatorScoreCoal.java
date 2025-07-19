package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Wrist.WristIO;

public class OperatorScoreCoal extends SequentialCommandGroup{
    private static SendableChooser<Integer> levelChooser;

    public OperatorScoreCoal(
        BooleanSupplier left,
        CoralEffector hand,
        WristIO wrist,
        ElevatorIO elevator,
        CoralFunnel funnel,
        DriveSubsystem drivetrain,
        IntSupplier goalTag
        ){
            IntSupplier levelSupplier = levelChooser::getSelected;
            SmartDashboard.putNumber("Operator Scoring Level", levelSupplier.getAsInt());
            if (levelSupplier.getAsInt() == 4) {
                addCommands(
                    new ParallelRaceGroup(
                        new MoveToScoringPosition(levelSupplier.getAsInt(), wrist, elevator),
                        new RunCommand(()->hand.intake(), hand)
                    ),
                    new MoveCoralToL4Position(levelSupplier.getAsInt(), hand),
                    new AlignToReefFieldRelative(left, drivetrain, levelSupplier, goalTag, elevator),
                    new WaitCommand(0.4),
                    new MoveToIntakePositions(wrist, elevator, funnel, hand).withTimeout(0.5)
                );
            } else {
                addCommands(
                    new ParallelRaceGroup(
                        new MoveToScoringPosition(levelSupplier.getAsInt(), wrist, elevator),
                        new RunCommand(()->hand.intake(), hand)
                    ),
                    new AlignToReefFieldRelative(left, drivetrain, levelSupplier, goalTag, elevator),
                    new ParallelCommandGroup(
                        new RunCommand(()->hand.outtake(), hand).withTimeout(1)
                    ),
                    new RunCommand(()->hand.stop(), hand).withTimeout(0.1),
                    new MoveToIntakePositions(wrist, elevator, funnel, hand).withTimeout(0.5)
                );
            }
            SmartDashboard.putString("Subsystems Required by Operator", this.getRequirements().toString());
    }

    public static void setSendableChooser(SendableChooser<Integer> chooser){
        levelChooser = chooser;
    }
}
