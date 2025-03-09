package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.FunnelConstants;
import frc.robot.Constants.WristConstants;
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

        super(
            
                Math.abs(wrist.getPosition() - WristConstants.IntakeAngle) < 0.5 ?
                    new MoveElevatorToLevel(0, elevator) :
                     new ParallelCommandGroup(
                        new RotateWristToLevel(4, wrist),
                        new MoveElevatorToLevel(0, elevator)
                     )
                    ,
                    
            new RotateWristToLevel(0, wrist),
            new RotateFunnel(funnel, FunnelConstants.IntakeAngle)
        );
    }
}
