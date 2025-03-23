package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Parser {
    
    private static final ArrayList<Command> defaultCommand = new ArrayList<>();
    // (1/2S)-(1-6)(1-4)(L/R)-(1/2C)-(1-6)(1-4)(L/R)-(1/2C)-(1-6)(1-4)(L/R)
    // (Start location)-(Side)(Level)(Pole)-(Coral station)-(Side)(Level)(Pole)-(Coral station)-(Side)(Level)(Pole)
    public static ArrayList<Command> parse(String input) {
        ArrayList<Command> commands = new ArrayList<>();
        int startPos;
        try {
            startPos = Integer.parseInt(input.substring(0, 1));
            SmartDashboard.putBoolean("Auto Parser Empty", false);
         }
         catch (NumberFormatException | IndexOutOfBoundsException e) {
            SmartDashboard.putBoolean("Auto Parser Empty", true);
            startPos = 1;
            return defaultCommand;
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
            SmartDashboard.putBoolean("Auto Loaded", true); 
        } catch (IndexOutOfBoundsException e) {
            SmartDashboard.putBoolean("Auto Loaded", false); 
            e.printStackTrace();
            return commands;
        }
        System.out.println(commands.size());
        return commands;
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
