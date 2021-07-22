package de.monticore.lang.gdl;

import java.util.List;

import org.apache.commons.collections4.list.UnmodifiableList;

public class Move {
    
    private String player;
    private List<String> arguments;

    private Move() {}

    public String getPlayer() {
        return player;
    }

    public List<String> getArguments() {
        return arguments;
    }

    public static Move createMoveFromLine(String line) {
        if (!line.matches("[a-zA-z0-9]+ \\(move ([a-zA-z0-9])+( [a-zA-z0-9]+)*\\)")) {
            System.out.println("Move format wrong");
            return null;
        }

        String[] split = line
            .replace("(", "")
            .replace(")", "")
            .split(" ");

        String role = split[0];
        String[] moveArgs = new String[split.length-2];
        for (int i = 0; i < moveArgs.length; i++) {
            moveArgs[i] = split[i+2];
        }
        
        Move move = new Move();
        move.player = role;
        move.arguments = UnmodifiableList.unmodifiableList(List.of(moveArgs));

        return move;
    }

}
