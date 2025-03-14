package frc.robot;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Parser {
    
    private static final ArrayList<Command> defaultCommand = parse("1S-13L-1C-63L-1C-63R");

    public static ArrayList<Command> parse(String input) {
        SmartDashboard.putString("Auto Parser Status", "Parsing Started");
        ArrayList<Command> commands = new ArrayList<>();
        int startPos;
        String[] steps = input.toUpperCase().split("-");
        try {
            if (steps.length < 2) return defaultCommand;
            startPos = Integer.parseInt(steps[0].substring(0, 1));
            commands.add(new SetPositionCommand(startPos));
            for (int i = 1; i < steps.length; i++) {
                commands.add(pathFromCode(steps[i - 1], steps[i]));
                if (steps[i].matches("[1-2]C")) {
                    commands.add(new GetCoralCommand());
                } else if (steps[i].matches("[1-6][1-4][LR]")) {
                    commands.add(new PutCoralCommand(steps[i].split("")));
                }
            }
        } catch (IndexOutOfBoundsException e) {
            SmartDashboard.putString("Auto Parser Status", "No Code Input");
            e.printStackTrace();
            return defaultCommand;
        } catch (IllegalArgumentException e) {
            SmartDashboard.putString("Auto Parser Status", "Invalid Code");
            e.printStackTrace();
            return defaultCommand;
        } catch (FileNotFoundException e) {
            SmartDashboard.putString("Auto Parser Status", "Path File Missing");
            e.printStackTrace();
            return defaultCommand;
        } catch (Exception e) {
            SmartDashboard.putString("Auto Parser Status", "Error");
            e.printStackTrace();
            return defaultCommand;
        }
        SmartDashboard.putString("Auto Parser Status", "Parsing Complete");
        return commands;
    }

    public static Command pathFromCode(String start, String end) throws FileVersionException, IllegalArgumentException, IOException, ParseException {
        if (!start.matches("[1-6][1-4][LR]|[1-2][CS]|3S") || !end.matches("[1-6][1-4][LR]|[1-2][CS]|3S")) {
            throw new IllegalArgumentException("Invalid auto code");
        }
        String startLoc = start.matches("[1-6][1-4][LR]") ? start.substring(0, 1) : start;
        String endLoc = end.matches("[1-6][1-4][LR]") ? end.substring(0, 1) : end;
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
    public static class SetPositionCommand extends Command {
        public int position;
        public SetPositionCommand(int position) {
            this.position = position;
        }
        public int getPosition(){
            return position - 1;
        }
    }
}
