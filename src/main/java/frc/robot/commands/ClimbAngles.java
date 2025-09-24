package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FunnelConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Wrist.WristIO;

public class ClimbAngles extends SequentialCommandGroup {
    public ClimbAngles(ElevatorIO elevator, WristIO wrist, Climber climber, CoralFunnel funnel){
        addRequirements(elevator, wrist, funnel, climber);
        addCommands(
            new ParallelCommandGroup(
                new MoveElevatorToLevel(0, elevator),
                new RotateWristToLevel(1, wrist),
                new RotateFunnel(funnel, FunnelConstants.ClimbAngle)
                // TODO: Add a command to move the climber to the climb position here
            )
        );
    }
}