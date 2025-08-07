package de.monticore.lang.gdl;

import de.monticore.lang.gdl.types.GDLTuple;
import de.monticore.lang.gdl.types.GDLType;

public class Command {
    
    private GDLTuple tuple;

    private Command() {
        tuple = null;
    }

    public GDLType getRole() {
        return tuple.get(0);
    }

    public GDLType getAction() {
        return tuple.get(1);
    }

    public boolean isNoop() {
        return tuple.size() > 1 && tuple.get(1).equals(new GDLTuple("noop"));
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
        command.tuple = GDLTuple.createFromPl("[" + plLine + "]");
        return command;
    }

    public static Command createFromGDLTuple(GDLTuple tuple) {
        Command command = new Command();
        command.tuple = tuple;
        return command;
    }

    @Override
    public boolean equals(Object obj){
        if(obj instanceof Command){
            if(this.getRole().equals(((Command)obj).getRole()) && this.getAction().equals(((Command)obj).getAction())){
                return true;
            } else {
                return false;
            }
        }

        return super.equals(obj);
    }

    @Override
    public int hashCode(){
        return this.getRole().hashCode() + this.getAction().hashCode();
    }
}
