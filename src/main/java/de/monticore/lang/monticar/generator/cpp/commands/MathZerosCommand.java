package de.monticore.lang.monticar.generator.cpp.commands;

/**
 * @author Sascha Schneiders
 */
public class MathZerosCommand extends MathInitCommand {
    public MathZerosCommand() {
        setMathCommandName("zeros");
    }

    @Override
    protected String getArmadilloInitCommandName() {
        return "zeros";
    }

    @Override
    protected String getOctaveInitCommandName() {
        return "Fzeros";
    }
}
