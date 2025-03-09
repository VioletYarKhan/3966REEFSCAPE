package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEffector;

public class MoveCoralToL4Position extends Command {
    public int level;
    public CoralEffector hand;

    public MoveCoralToL4Position(int level, CoralEffector hand){

        this.level = level;
        this.hand = hand;

        addRequirements(hand);
    }

    @Override
    public void initialize(){
        SmartDashboard.putBoolean(getName(), true);
        hand.goToPosition(level);
    }

    @Override
    public boolean isFinished() {
        return (hand.atTarget(1));
    }

    @Override
    public void end(boolean interrupt){
        SmartDashboard.putBoolean(getName(), false);
    }
}
