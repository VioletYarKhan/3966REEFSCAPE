package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Wrist.WristIO;

public class WristIntakePosition extends Command {
    public WristIO wrist;
    public CoralEffector hand;


    public WristIntakePosition(
        CoralEffector hand,
        WristIO wrist){

        this.hand = hand;
        this.wrist = wrist;

        addRequirements(wrist.returnSubsystem());
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
