package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralFunnel;

public class RotateFunnel extends Command {
    private final coralFunnel m_funnel;
    private boolean up;

    public RotateFunnel(coralFunnel funnel, double setpoint){
        this.m_funnel = funnel;

        addRequirements(m_funnel);
    }

    @Override
    public void initialize() {
        if (m_funnel.getPosition() < 0.005 || m_funnel.getPosition() > 0.995){
            up = true;
        } else {
            up = false;
        }
    }

    @Override
    public void execute() {
        if(up){
            m_funnel.set(-0.2);
        } else {
            m_funnel.set(0.2);
        }
    }

    @Override
    public boolean isFinished() {
        if (up){
            return (m_funnel.getPosition() < 0.005 || m_funnel.getPosition() > 0.995);
        } else {
            return (m_funnel.getPosition() < 0.325 || m_funnel.getPosition() > 0.315);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_funnel.set(0);
    }
}
