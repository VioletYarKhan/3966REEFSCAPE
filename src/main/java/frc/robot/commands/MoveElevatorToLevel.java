package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class MoveElevatorToLevel extends Command {
    public int level;
    public Elevator elevator;

    public MoveElevatorToLevel(int level, Elevator elevator){

        this.level = level;
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize(){
        SmartDashboard.putBoolean("Elevator Command", true);
        if (level == 0){
            elevator.setPosition(ElevatorConstants.IntakeHeight);
        } else if(level == 1){
            elevator.setPosition(ElevatorConstants.L1Height);
        } else if(level == 2){
            elevator.setPosition(ElevatorConstants.L2Height);
        } else if(level == 3){
            elevator.setPosition(ElevatorConstants.L3Height);
        } else if(level == 4){
            elevator.setPosition(ElevatorConstants.L4Height);
        }    
    }

    @Override
    public boolean isFinished() {
        return (elevator.atTarget(1));
    }

    @Override
    public void end(boolean interrupt){
        SmartDashboard.putBoolean("Elevator Command", false);
    }
}
