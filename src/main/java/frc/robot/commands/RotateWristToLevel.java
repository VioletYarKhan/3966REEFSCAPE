package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.effectorWrist;

public class RotateWristToLevel extends Command {
    public int level;
    public effectorWrist wrist;


    public RotateWristToLevel(
        int level,
        effectorWrist wrist){

        this.level = level;
        this.wrist = wrist;

        addRequirements(wrist);
    }

    @Override
    public void execute(){
        if(level == 1){
            wrist.setPosition(WristConstants.IntakeAngle);
        } else if(level == 1){
            wrist.setPosition(WristConstants.L1Angle);
        } else if(level == 2 || level == 3){
            wrist.setPosition(WristConstants.L2_3Angle);
        } else if(level == 4){
            wrist.setPosition(WristConstants.L4Angle);
        }     
    }

    @Override
    public boolean isFinished() {
        return (wrist.atTarget(0.1));
    }
}
