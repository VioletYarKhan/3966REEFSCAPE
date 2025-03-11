package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.EffectorWrist;

public class Parser {
    
    private static final SequentialCommandGroup defaultCommand = new SequentialCommandGroup();
    // (1/2S)-(1-6)(1-4)(L/R)-(1/2C)-(1-6)(1-4)(L/R)-(1/2C)-(1-6)(1-4)(L/R)
    // (Start location)-(Side)(Level)(Pole)-(Coral station)-(Side)(Level)(Pole)-(Coral station)-(Side)(Level)(Pole)
    public static SequentialCommandGroup parse(String input, DriveSubsystem drivetrain, CoralEffector hand, EffectorWrist wrist, Elevator elevator, CoralFunnel funnel) {
        SequentialCommandGroup commands = new SequentialCommandGroup();
        String[] steps = input.split("-");
        try {
            for (int i = 0; i < 5; i += 2) {
                commands.addCommands(pathFromCode(steps[i], steps[i + 1].substring(0, 1)));
                commands.addCommands(scoreCoral(steps[i + 1].split(""), drivetrain, hand, wrist, elevator, funnel));
                if (i < 4) {
                    commands.addCommands(pathFromCode(steps[i + 1].substring(0, 1), steps[i + 2]));
                    commands.addCommands(new GetCoralCommand());
                }
            }
        } catch (Exception e) {
            return defaultCommand;
        }
        return commands;
    }

    private static Command pathFromCode(String start, String end) throws FileVersionException, IOException, ParseException {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(start + "-" + end));
    }

    private static ScoreCoral scoreCoral(String[] code, DriveSubsystem drivetrain, CoralEffector hand, EffectorWrist wrist, Elevator elevator, CoralFunnel funnel) {
        return new ScoreCoral(Integer.parseInt(code[1]), code[2].equalsIgnoreCase("R"), hand, wrist, elevator, funnel, drivetrain);
    }

    private static class GetCoralCommand extends Command {}
}
