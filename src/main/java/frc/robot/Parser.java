package frc.robot;

import java.io.IOException;
import java.util.ArrayList;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;

public class Parser {
    
    private static final ArrayList<Command> defaultCommand = new ArrayList<>();
    public static ArrayList<Command> parse(String input) {
        ArrayList<Command> commands = new ArrayList<>();
        String[] steps = input.toUpperCase().split("-");
        try {
            if (steps.length < 2) return defaultCommand;
            
            for (int i = 1; i < steps.length; i++) {
                commands.add(pathFromCode(steps[i - 1], steps[i]));
                if (steps[i].matches("[1-2]C")) {
                    commands.add(new GetCoralCommand());
                } else if (steps[i].matches("[1-6][1-4][LR]")) {
                    commands.add(new PutCoralCommand(steps[i].split("")));
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
            return defaultCommand;
        }
        return commands;
    }

    public static Command pathFromCode(String start, String end) throws FileVersionException, IOException, ParseException {
        String startLoc = start.matches("\\d\\d[LR]") ? start.substring(0, 1) : start;
        String endLoc = end.matches("\\d\\d[LR]") ? end.substring(0, 1) : end;
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(startLoc + "-" + endLoc));
    }

    // Placeholder
    public static class PutCoralCommand extends Command {
        public int side;
        public int level;
        public boolean left;
        public PutCoralCommand(String[] code) {
            this.side = Integer.parseInt(code[0]);
            this.level = Integer.parseInt(code[1]);
            this.left = code[2].equals("L");
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
