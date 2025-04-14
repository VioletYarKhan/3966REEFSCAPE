package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Wrist.WristIO;
import frc.robot.Constants.FunnelConstants;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.Elevator.ElevatorIO;

public class MoveToIntakePositions extends SequentialCommandGroup {
    public ElevatorIO elevator;
    public WristIO wrist;
    public CoralFunnel funnel;
    public CoralEffector hand;


    public MoveToIntakePositions(
        WristIO wrist,
        ElevatorIO elevator,
        CoralFunnel funnel,
        CoralEffector hand){

        addCommands(new MoveElevatorToLevel(0, elevator), new WristIntakePosition(hand, wrist));
        addCommands(new RotateFunnel(funnel, FunnelConstants.IntakeAngle));
    
    }
}
