package de.monticore.lang.monticar.generator.cpp.commands;

/**
 * @author Sascha Schneiders
 */
public class MathOnesCommand extends MathInitCommand {
    public MathOnesCommand() {
        setMathCommandName("ones");
    }

    @Override
    protected String getArmadilloInitCommandName() {
        return "ones";
    }

    @Override
    protected String getOctaveInitCommandName() {
        return "Fones";
    }
}
