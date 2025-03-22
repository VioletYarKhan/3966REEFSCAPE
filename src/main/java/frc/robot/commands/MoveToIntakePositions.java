package frc.robot.commands;


import java.util.function.BooleanSupplier;

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
    public boolean hasCoral;


    public MoveToIntakePositions(
        EffectorWrist wrist,
        Elevator elevator,
        CoralFunnel funnel,
        BooleanSupplier hasCoral){

        if (!hasCoral.getAsBoolean()){
            addCommands(new MoveElevatorToLevel(0, elevator).andThen(new RotateWristToLevel(0, wrist)));
        } else {
            addCommands(new ParallelCommandGroup(
                new RotateWristToLevel(4, wrist),
                new MoveElevatorToLevel(0, elevator)
            ));
        }
        addCommands(new RotateFunnel(funnel, FunnelConstants.IntakeAngle));
    
    }
}
