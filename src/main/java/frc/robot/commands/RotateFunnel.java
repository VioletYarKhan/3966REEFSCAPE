package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralFunnel;

public class RotateFunnel extends Command {
    private final CoralFunnel m_funnel;
    private final double setpoint;
    public RotateFunnel(CoralFunnel funnel, double setpoint){
        this.m_funnel = funnel;
        this.setpoint = setpoint;
        addRequirements(m_funnel);
    }

    @Override
    public void execute() {
        if(m_funnel.getPosition() > setpoint){
            m_funnel.set(0.2);
        } else {
            m_funnel.set(-0.2);
        }
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(m_funnel.getPosition() - setpoint) < 0.005);
    }

    @Override
    public void end(boolean interrupted) {
        m_funnel.set(0);
    }
}
