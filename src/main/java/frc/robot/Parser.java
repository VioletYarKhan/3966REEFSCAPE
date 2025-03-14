package frc.robot;

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
        ArrayList<Command> commands = new ArrayList<>();
        int startPos;
        String[] steps = input.toUpperCase().split("-");
        try {
            startPos = Integer.parseInt(input.substring(0, 1));
            SmartDashboard.putBoolean("Auto Parser Empty", false);
            commands.add(new SetPositionCommand(startPos));
            for (int i = 0; i < 5; i += 2) {
                commands.add(pathFromCode(steps[i], steps[i + 1].substring(0, 1)));
                commands.add(new PutCoralCommand(steps[i + 1].split("")));
                if (i < 4) {
                    commands.add(pathFromCode(steps[i + 1].substring(0, 1), steps[i + 2]));
                    commands.add(new GetCoralCommand());
                }
            }
        } catch (NumberFormatException | IndexOutOfBoundsException e) {
            SmartDashboard.putBoolean("Auto Parser Empty", true);
            return defaultCommand;
        } catch (Exception e) {
            e.printStackTrace();
            return defaultCommand;
        }
        System.out.println(commands.size());
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
