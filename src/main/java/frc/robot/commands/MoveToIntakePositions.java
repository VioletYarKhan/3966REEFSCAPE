package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.FunnelConstants;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.EffectorWrist;

public class MoveToIntakePositions extends SequentialCommandGroup {
    public Elevator elevator;
    public EffectorWrist wrist;
    public CoralFunnel funnel;
    public CoralEffector hand;


    public MoveToIntakePositions(
        EffectorWrist wrist,
        Elevator elevator,
        CoralFunnel funnel,
        CoralEffector hand){

            addCommands(new MoveElevatorToLevel(0, elevator).andThen(new RotateWristToLevel(0, wrist)));
            addCommands(new ParallelCommandGroup(
                new MoveElevatorToLevel(0, elevator)
            ));
        addCommands(new RotateFunnel(funnel, FunnelConstants.IntakeAngle));
    
    }
}
