package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.FunnelConstants;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.EffectorWrist;

public class MoveToIntakePositions extends SequentialCommandGroup {
    public Elevator elevator;
    public EffectorWrist wrist;
    public CoralFunnel funnel;


    public MoveToIntakePositions(
        EffectorWrist wrist,
        Elevator elevator,
        CoralFunnel funnel){

        if (wrist.getPosition()< 1){
            addCommands(new MoveElevatorToLevel(0, elevator));
        } else {
            addCommands(new ParallelCommandGroup(
                new RotateWristToLevel(4, wrist),
                new MoveElevatorToLevel(0, elevator)
            ));
        }
        addCommands(new RotateWristToLevel(0, wrist),
        new RotateFunnel(funnel, FunnelConstants.IntakeAngle));
    
    }
}
