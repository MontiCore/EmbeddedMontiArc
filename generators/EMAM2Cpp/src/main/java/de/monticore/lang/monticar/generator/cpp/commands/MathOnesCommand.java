/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.commands;

/**
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
