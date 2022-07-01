package de.monticore.lang.gdl;

import de.monticore.lang.gdl.types.GDLTuple;
import de.monticore.lang.gdl.types.GDLType;

public class Command {
    
    private GDLTuple tuple;

    private Command() {
        tuple = null;
    }

    @Override
    public String toString() {
        String tupleString = tuple.toString();
        return tupleString.substring(1, tupleString.length() - 1);
    }
        
    public String toPlString() {
        String tuplePlString = tuple.toPlString();
        return tuplePlString;
    }

    public static Command createFromLine(String line) {
        Command command = new Command();
        command.tuple = GDLTuple.createFromLine("(" + line + ")");
        return command;
    }

    public static Command createFromPl(String plLine) {
        Command command = new Command();
        command.tuple = GDLTuple.createFromPl("(" + plLine + ")");
        return command;
    }

    public static Command createFromGDLTuple(GDLTuple tuple) {
        Command command = new Command();
        command.tuple = tuple;
        return command;
    }

    public static Command createFromGDLPlTuple(GDLTuple tuple) {
        Command command = new Command();

        GDLType[] args = tuple.getElements().stream().skip(1).toArray(i -> new GDLType[i]);

        command.tuple = new GDLTuple(tuple.getElements().get(0), new GDLTuple(args));
        return command;
    }

}
