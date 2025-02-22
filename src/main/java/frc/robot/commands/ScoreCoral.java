package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.GryphonLib.PositionCalculations;
import frc.robot.Vision;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.coralEffector;
import frc.robot.subsystems.effectorWrist;

public class ScoreCoral extends Command {
    public int level;
    public boolean left;
    public DriveSubsystem drivetrain;
    public Elevator elevator;
    public coralEffector coralHand;
    public effectorWrist wrist;

    private Command outTake = new InstantCommand(()->coralHand.outtake(), coralHand).withTimeout(0.5);

    public ScoreCoral(
        int level,
        boolean left,
        DriveSubsystem drivetrain,
        coralEffector hand,
        effectorWrist wrist,
        Elevator elevator){

        this.level = level;
        this.left = left;
        this.drivetrain = drivetrain;
        this.coralHand = hand;
        this.wrist = wrist;
        this.elevator = elevator;

        addRequirements(drivetrain, wrist, elevator);
    }

    @Override
    public void execute(){
        if(level == 1){
            wrist.setPosition(WristConstants.L1Angle);
        } else if(level == 2){
            wrist.setPosition(WristConstants.L2_3Angle);
        } else if(level == 3){
            wrist.setPosition(WristConstants.L2_3Angle);
        } else if(level == 4){
            wrist.setPosition(WristConstants.L4Angle);
        }
        if (wrist.atTarget(0.1)){
            if(level == 1){
                elevator.setPosition(ElevatorConstants.L1Height);
            } else if(level == 2){
                elevator.setPosition(ElevatorConstants.L2Height);
            } else if(level == 3){
                elevator.setPosition(ElevatorConstants.L3Height);
            } else if(level == 4){
                elevator.setPosition(ElevatorConstants.L4Height);
            }
        }

        drivetrain.PathToPose(PositionCalculations.getGoalPoseFromTag(Vision.getCamera(), drivetrain.getCurrentPose(), left ? AutoConstants.leftBranchCoral : AutoConstants.rightBranchCoral, Vision.getBestTag()));
        
    }

    @Override
    public boolean isFinished() {
        return (wrist.atTarget(0.1) && elevator.atTarget(0.1));
    }

    @Override
    public void end(boolean interrupt){
        outTake.schedule();
    }
}
