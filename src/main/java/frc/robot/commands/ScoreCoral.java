package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GryphonLib.PositionCalculations;
import frc.robot.Vision;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.coralEffector;
import frc.robot.subsystems.effectorWrist;

public class ScoreCoral extends SequentialCommandGroup{
    public int level;
    public boolean left;
    public DriveSubsystem drivetrain;
    public Elevator elevator;
    public coralEffector coralHand;
    public effectorWrist wrist;


    public ScoreCoral(
        int level,
        boolean left,
        DriveSubsystem drivetrain,
        coralEffector hand,
        effectorWrist wrist,
        Elevator elevator){

        super(
            drivetrain.PathToPose(PositionCalculations.getGoalPoseFromTag(Vision.getCamera(), drivetrain.getCurrentPose(), left ? AutoConstants.leftBranchCoral : AutoConstants.rightBranchCoral, Vision.getBestTag())),
            new MoveElevatorToLevel(level, wrist, elevator),
            new RotateWristToLevel(level, wrist),
            new InstantCommand(()-> hand.outtake(), hand).withTimeout(0.5),
            new MoveElevatorToLevel(0, wrist, elevator)
        );
    }
}
