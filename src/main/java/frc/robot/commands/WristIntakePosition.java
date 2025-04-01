package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.EffectorWrist;

public class WristIntakePosition extends Command {
    public EffectorWrist wrist;
    public CoralEffector hand;


    public WristIntakePosition(
        CoralEffector hand,
        EffectorWrist wrist){

        this.hand = hand;
        this.wrist = wrist;

        addRequirements(wrist);
    }

    @Override
    public void initialize(){
        if(hand.hasCoral() && wrist.getPosition() > 3){
            wrist.setPosition(WristConstants.L4Angle);
        } else{
            wrist.setPosition(WristConstants.IntakeAngle);   
        } 
    }

    @Override
    public boolean isFinished() {
        return (wrist.atTarget(1));
    }

    @Override
    public void end(boolean interrupted) {
    }
}
