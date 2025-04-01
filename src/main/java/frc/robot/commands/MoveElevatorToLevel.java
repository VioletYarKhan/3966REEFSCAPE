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
                
        if (level == 0){
            elevator.setPosition(SmartDashboard.getNumber("Elevator Intake Height", ElevatorConstants.IntakeHeight));
        } else if(level == 1){
            elevator.setPosition(SmartDashboard.getNumber("Elevator L1 Height", ElevatorConstants.L1Height));
        } else if(level == 2){
            elevator.setPosition(SmartDashboard.getNumber("Elevator L2 Height", ElevatorConstants.L2Height));
        } else if(level == 3){
            elevator.setPosition(SmartDashboard.getNumber("Elevator L3 Height", ElevatorConstants.L3Height));
        } else if(level == 4){
            elevator.setPosition(SmartDashboard.getNumber("Elevator L4 Height", ElevatorConstants.L4Height));
        }    
    }

    @Override
    public boolean isFinished() {
        return (elevator.atTarget(2));
    }

    @Override
    public void end(boolean interrupt){
    }
}
