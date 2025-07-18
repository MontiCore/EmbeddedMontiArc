/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.commands;

/**
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
