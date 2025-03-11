package frc.robot;

import java.io.IOException;
import java.util.ArrayList;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;

public class Parser {
    
    private static final ArrayList<Command> defaultCommand = new ArrayList<Command>();
    // (1/2S)-(1-6)(1-4)(L/R)-(1/2C)-(1-6)(1-4)(L/R)-(1/2C)-(1-6)(1-4)(L/R)
    // (Start location)-(Side)(Level)(Pole)-(Coral station)-(Side)(Level)(Pole)-(Coral station)-(Side)(Level)(Pole)
    public static ArrayList<Command> parse(String input) {
        ArrayList<Command> commands = new ArrayList<Command>();
        String[] steps = input.split("-");
        try {
            for (int i = 0; i < 5; i += 2) {
                commands.add(pathFromCode(steps[i], steps[i + 1].substring(0, 1)));
                commands.add(new PutCoralCommand(steps[i + 1].split("")));
                if (i < 4) {
                    commands.add(pathFromCode(steps[i + 1].substring(0, 1), steps[i + 2]));
                    commands.add(new GetCoralCommand());
                }
            }
        } catch (Exception e) {
            return defaultCommand;
        }
        return commands;
    }

    public static Command pathFromCode(String start, String end) throws FileVersionException, IOException, ParseException {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(start + "-" + end));
    }

    // Placeholder
    public static class PutCoralCommand extends Command {
        public int side;
        public int level;
        public boolean left;
        public PutCoralCommand(int side, int level, boolean left) {
            this.side = side;
            this.level = level;
            this.left = left;
        }
        public PutCoralCommand(String[] code) {
            this(Integer.parseInt(code[0]), Integer.parseInt(code[1]), code[2].equalsIgnoreCase("L"));
        }
        public int getLevel(){
            return level;
        }
        public boolean getLeft(){
            return left;
        }
    }
    public static class GetCoralCommand extends Command {}
}
