package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.GryphonLib.PositionCalculations;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Wrist.WristIO;

public class OperatorScoreCoal extends SequentialCommandGroup{
    public OperatorScoreCoal(
        boolean left,
        CoralEffector hand,
        WristIO wrist,
        ElevatorIO elevator,
        CoralFunnel funnel,
        DriveSubsystem drivetrain,
        int goalTag){
            @SuppressWarnings("unchecked")
            SendableChooser<Integer> levelChooser = (SendableChooser<Integer>) SmartDashboard.getData("Operator Height Chooser");
            int level = levelChooser.getSelected();
            SmartDashboard.putNumber("Operator Scoring Level", level);
            if (level == 4) {
                addCommands(
                    new MoveToScoringPosition(level, wrist, elevator),
                    new MoveCoralToL4Position(level, hand),
                    drivetrain.PathToPose(PositionCalculations.getAlignmentReefPose(goalTag, level, left), 0.0),
                    new WaitCommand(0.4),
                    new MoveToIntakePositions(wrist, elevator, funnel, hand).withTimeout(0.5)
                );
            } else {
                addCommands(
                    new MoveToScoringPosition(level, wrist, elevator),
                    drivetrain.PathToPose(PositionCalculations.getAlignmentReefPose(goalTag, level, left), 0.0),
                    new ParallelCommandGroup(
                        new RunCommand(()->hand.outtake(), hand).withTimeout(1)
                    ),
                    new RunCommand(()->hand.stop(), hand).withTimeout(0.1)
                );
            }
            SmartDashboard.putString("Subsystems Required by Operator", this.getRequirements().toString());
    }
}
