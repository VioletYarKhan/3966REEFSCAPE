package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EffectorWrist;

public class MoveToIntakePositions extends SequentialCommandGroup {
    public Elevator elevator;
    public EffectorWrist wrist;


    public MoveToIntakePositions(
        EffectorWrist wrist,
        Elevator elevator){

        super(
            new MoveElevatorToLevel(0, elevator),
            new RotateWristToLevel(0, wrist)
        );
    }
}
