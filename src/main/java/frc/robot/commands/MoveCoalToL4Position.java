package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEffector;

public class MoveCoalToL4Position extends Command {
    public CoralEffector hand;
    public int level;
    public double reqPos;

    public MoveCoalToL4Position(int level, CoralEffector hand){
        this.level = level;
        this.hand = hand;

        addRequirements(hand);
    }

    @Override
    public void initialize(){
        reqPos = hand.getPosition()-0.95;
    }

    @Override
    public void execute(){
        if(level == 4){
            hand.setPosition(reqPos);
        } else {
            hand.setPosition(hand.getPosition());
        }
    }

    @Override
    public boolean isFinished() {
        return (hand.atTarget(1));
    }

    @Override
    public void end(boolean interrupt){
    }
}