package de.monticore.lang.gdl;

import java.util.List;

import org.apache.commons.collections4.list.UnmodifiableList;

public class Command {
    
    private String player;
    private List<String> arguments;

    private Command() {}

    public String getPlayer() {
        return player;
    }

    public List<String> getArguments() {
        return arguments;
    }

    public static Command createMoveFromLine(String line) {
        if (!line.matches("[a-zA-z0-9]+ \\(([a-zA-z0-9])+( [a-zA-z0-9]+)*\\)")) {
            System.out.println("Move format wrong");
            return null;
        }

        String[] split = line
            .replace("(", "")
            .replace(")", "")
            .split(" ");

        String role = "value_" + split[0];
        String[] moveArgs = new String[split.length-1];
        for (int i = 0; i < moveArgs.length; i++) {
            moveArgs[i] = "value_" + split[i+1];
        }
        
        Command move = new Command();
        move.player = role;
        move.arguments = UnmodifiableList.unmodifiableList(List.of(moveArgs));

        return move;
    }

}
