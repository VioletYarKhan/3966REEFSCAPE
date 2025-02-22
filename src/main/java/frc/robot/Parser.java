package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Parser {
    
    private static final SequentialCommandGroup defaultCommand = new SequentialCommandGroup();
    // (1/2S)-(1-6)(1-4)(L/R)-(1/2C)-(1-6)(1-4)(L/R)-(1/2C)-(1-6)(1-4)(L/R)
    // (Start location)-(Side)(Level)(Pole)-(Coral station)-(Side)(Level)(Pole)-(Coral station)-(Side)(Level)(Pole)
    public static SequentialCommandGroup parse(String input) {
        SequentialCommandGroup commands = new SequentialCommandGroup();
        String[] steps = input.split("-");
        try {
            for (int i = 0; i < 5; i += 2) {
                commands.addCommands(pathFromCode(steps[i], steps[i + 1].substring(0, 1)));
                commands.addCommands(new PutCoralCommand(steps[i + 1].split("")));
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

    // Placeholder
    private static class PutCoralCommand extends Command {
        public PutCoralCommand(int side, int level, boolean left) {}
        public PutCoralCommand(String[] code) {
            this(Integer.parseInt(code[0]), Integer.parseInt(code[1]), code[2].equalsIgnoreCase("L"));
        }
    }
    private static class GetCoralCommand extends Command {}
}
