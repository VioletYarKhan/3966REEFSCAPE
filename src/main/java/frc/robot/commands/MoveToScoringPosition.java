package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EffectorWrist;
import frc.robot.subsystems.Elevator;

public class MoveToScoringPosition extends SequentialCommandGroup {
    public Elevator elevator;
    public int level;
    public EffectorWrist wrist;

    public MoveToScoringPosition(
        int level,
        EffectorWrist wrist,
        Elevator elevator
       ){

        super(
            new RotateWristToLevel(level, wrist),
            new MoveElevatorToLevel(level, elevator)
        );
    }
}
