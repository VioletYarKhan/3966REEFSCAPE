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
    
    private static final ArrayList<Command> defaultCommand = parse("1S-24L");
    // (1/2S)-(1-6)(1-4)(L/R)-(1/2C)-(1-6)(1-4)(L/R)-(1/2C)-(1-6)(1-4)(L/R)
    // (Start location)-(Side)(Level)(Pole)-(Coral station)-(Side)(Level)(Pole)-(Coral station)-(Side)(Level)(Pole)
    public static ArrayList<Command> parse(String input) {
        SmartDashboard.putString("Auto Parser Status", "Parsing Started");
        ArrayList<Command> commands = new ArrayList<>();
        int startPos;
        try {
            startPos = Integer.parseInt(input.substring(0, 1));
         }
         catch (NumberFormatException | IndexOutOfBoundsException e) {
            SmartDashboard.putString("Auto Parser Status", "No Code Input");
            startPos = 1;
            return new ArrayList<Command>();
         }
        commands.add(new SetPositionCommand(startPos));
        String[] steps = input.split("-");
        try {
            for(int i = 0; i < steps.length - 1; i++){
                if(i%2 == 0){
                    commands.add(new PutCoralCommand(steps[i + 1].split("")));
                } else {
                    commands.add(new GetCoralCommand(steps[i + 1].split("")));
                }      
            }     
        } catch (IndexOutOfBoundsException e) {
            SmartDashboard.putString("Auto Parser Status", "Parsing Started");
            e.printStackTrace();
            defaultCommand.remove(0);
            defaultCommand.add(0, new SetPositionCommand(startPos));
            return defaultCommand;
        } catch (IllegalArgumentException e) {
            SmartDashboard.putString("Auto Parser Status", "Invalid Code");
            e.printStackTrace();
            defaultCommand.remove(0);
            defaultCommand.add(0, new SetPositionCommand(startPos));
            return defaultCommand;
        }  catch (Exception e) {
            SmartDashboard.putString("Auto Parser Status", "Error");
            e.printStackTrace();
            defaultCommand.remove(0);
            defaultCommand.add(0, new SetPositionCommand(startPos));
            return defaultCommand;
        }
        SmartDashboard.putString("Auto Parser Status", "Parsing Complete");
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
        public PutCoralCommand(String[] code) {
            this.side = Integer.parseInt(code[0]);
            this.level = Integer.parseInt(code[1]);
            this.left = code[2].equalsIgnoreCase("L");
        }
        public int getLevel(){
            return level;
        }
        public int getSide(){
            return side;
        }
        public boolean getLeft(){
            return left;
        }
    }

    public static class GetCoralCommand extends Command {
        public int station;

        public GetCoralCommand(String[] code){
            this.station = Integer.parseInt(code[0]);
        }

        public int getStation(){
            return station;
        }
    }

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
