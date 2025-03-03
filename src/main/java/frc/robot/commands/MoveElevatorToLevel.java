package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EffectorWrist;

public class MoveElevatorToLevel extends Command {
    public int level;
    public Elevator elevator;
    public EffectorWrist wrist;

    public MoveElevatorToLevel(
        int level,
        EffectorWrist wrist,
        Elevator elevator){

        this.level = level;
        this.wrist = wrist;
        this.elevator = elevator;

        addRequirements(wrist, elevator);
    }

    @Override
    public void execute(){
        wrist.setPosition(WristConstants.L4Angle);
        if (wrist.atTarget(0.1)){
            if (level == 0){
                elevator.setPosition(ElevatorConstants.IntakeHeight);
            }else if(level == 1){
                elevator.setPosition(ElevatorConstants.L1Height);
            } else if(level == 2){
                elevator.setPosition(ElevatorConstants.L2Height);
            } else if(level == 3){
                elevator.setPosition(ElevatorConstants.L3Height);
            } else if(level == 4){
                elevator.setPosition(ElevatorConstants.L4Height);
            }
        }        
    }

    @Override
    public boolean isFinished() {
        return (wrist.atTarget(0.1) && elevator.atTarget(0.1));
    }

    @Override
    public void end(boolean interrupt){}
}
