package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Wrist.WristIO;

public class RotateWristToLevel extends Command {
    public int level;
    public WristIO wrist;


    public RotateWristToLevel(
        int level,
        WristIO wrist){

        this.level = level;
        this.wrist = wrist;

        addRequirements(wrist.returnSubsystem());
    }

    @Override
    public void initialize(){
        if(level == 0){
            wrist.setPosition(SmartDashboard.getNumber("Wrist Intake Angle", WristConstants.IntakeAngle));
        } else if(level == 1){
            wrist.setPosition(SmartDashboard.getNumber("Wrist L1 Angle", WristConstants.L1Angle));
        } else if(level == 2 || level == 3){
            wrist.setPosition(SmartDashboard.getNumber("Wrist L2-3 Angle", WristConstants.L2_3Angle));
        } else if(level == 4){
            wrist.setPosition(SmartDashboard.getNumber("Wrist L4 Angle", WristConstants.L4Angle));
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
