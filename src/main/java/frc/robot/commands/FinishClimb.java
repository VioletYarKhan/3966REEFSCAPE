package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

public class FinishClimb extends SequentialCommandGroup {
    public FinishClimb(Climber climber){
        addRequirements(climber);
        addCommands(new RunCommand(()->climber.setPosition(10), climber));
    }
}
