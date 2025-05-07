package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Wrist.WristIO;

public class MoveToScoringPosition extends SequentialCommandGroup {
    public ElevatorIO elevator;
    public int level;
    public WristIO wrist;

    public MoveToScoringPosition(
        int level,
        WristIO wrist,
        ElevatorIO elevator
       ){

        SmartDashboard.putNumber("Scored level", level);
        addCommands(new RotateWristToLevel(level, wrist), new MoveElevatorToLevel(level, elevator));
        
    }
}
