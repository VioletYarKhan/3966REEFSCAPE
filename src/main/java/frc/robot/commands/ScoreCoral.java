package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.EffectorWrist;

public class ScoreCoral extends SequentialCommandGroup{
    public ScoreCoral(
        int level,
        CoralEffector hand,
        EffectorWrist wrist,
        Elevator elevator,
        CoralFunnel funnel){

        super(
            new MoveToScoringPosition(level, wrist, elevator),
            level == 4 ? new RunCommand(()->hand.setPosition(hand.getPosition() + 0.15), hand).until(()->hand.atTarget(0.1)) : new RunCommand(()->hand.outtake(), hand).withTimeout(3),
            new MoveToIntakePositions(wrist, elevator, funnel)
        );
    }
}
