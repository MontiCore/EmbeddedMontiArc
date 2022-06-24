package de.monticore.lang.gdl;

import java.util.List;
import java.util.stream.Collectors;

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
        sb.append(this.getPlayer().substring(6) + " ");
        sb.append("(");
        for (int i = 0; i < this.getArguments().size(); i++) {

            if (i < (this.getArguments().size() - 1)) {
                sb.append(Interpreter.convertPLValue2Interpreter(this.getArguments().get(i)) + " ");
            } else {
                sb.append(Interpreter.convertPLValue2Interpreter(this.getArguments().get(i)));
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
        list = list.stream().map(s -> s.startsWith("value_") || s.startsWith("valnn_") ? s : Interpreter.convertInterpreterValue2PL(s)).collect(Collectors.toList());

        move.player = list.get(0);
        move.arguments = list.stream().skip(1).collect(Collectors.toList());

        return move;
    }

    public static Command createMoveFromLine(String line) {
        String pattern = "([a-zA-z0-9]|\\-)+ \\(([a-zA-z0-9]|\\-)+( ([a-zA-z0-9]|\\-)+)*\\)";
        if (!line.matches(pattern)) {
            System.out.println("Move format wrong");
            return null;
        }

        String[] split = line
            .replace("(", "")
            .replace(")", "")
            .split(" ");

        String role = Interpreter.convertInterpreterValue2PL(split[0]);
        String[] moveArgs = new String[split.length-1];
        for (int i = 0; i < moveArgs.length; i++) {
            moveArgs[i] = Interpreter.convertInterpreterValue2PL(split[i+1]);
        }
        
        Command move = new Command();
        move.player = role;
        move.arguments = UnmodifiableList.unmodifiableList(List.of(moveArgs));

        return move;
    }

}
