package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GryphonLib.PositionCalculations;
import frc.robot.Vision;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.EffectorWrist;

public class ScoreCoral extends SequentialCommandGroup{
    public int level;
    public boolean left;
    public DriveSubsystem drivetrain;
    public Elevator elevator;
    public CoralEffector coralHand;
    public EffectorWrist wrist;


    public ScoreCoral(
        int level,
        CoralEffector hand,
        EffectorWrist wrist,
        Elevator elevator){

        super(
            new RotateWristToLevel(level, wrist),
            new MoveElevatorToLevel(level, elevator),
            new InstantCommand(()-> hand.outtake(), hand).withTimeout(0.5),
            new MoveToIntakePositions(wrist, elevator)
        );
    }
}
