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

    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(this.getPlayer() + " ");
        sb.append("(");
        for (int i = 0; i < this.getArguments().size(); i++) {

            if (i < (this.getArguments().size() - 1)) {
                sb.append(this.getArguments().get(i) + " ");
            } else {
                sb.append(this.getArguments().get(i));
            }
        }
        sb.append(")");

        return sb.toString();
    }

    public static Command createMoveFromList(List<String> list) {


        if (list.size() < 2) {
            return null;
        }
        
        Command move = new Command();
        move.player = list.get(0);
        move.arguments = UnmodifiableList.unmodifiableList(list.subList(1, list.size()));

        return move;
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
