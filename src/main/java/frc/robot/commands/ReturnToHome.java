package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Wrist.WristIO;

public class ReturnToHome extends SequentialCommandGroup {
    public ReturnToHome(ElevatorIO elevator, WristIO wrist, Climber climber, CoralFunnel funnel, CoralEffector hand){
        addCommands(
            new ParallelCommandGroup(
                new MoveToIntakePositions(wrist, elevator, funnel, hand)
                // TODO: Add a command to move the climber to the intake position here
            )
        );
    }
}