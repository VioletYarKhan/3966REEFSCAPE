package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.EffectorWrist;

public class RemoveAlgae extends Command {
    public int level;
    public boolean left;
    public DriveSubsystem drivetrain;
    public Elevator elevator;
    public CoralEffector coralHand;
    public EffectorWrist wrist;
}
