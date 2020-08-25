/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

/**
 */
public class TargetCodeInstruction implements Instruction {
    protected String instruction;

    public TargetCodeInstruction() {

    }

    public TargetCodeInstruction(String instruction) {
        this.instruction = instruction;
    }


    public String getInstruction() {
        return instruction;
    }

    public void setInstruction(String instruction) {
        this.instruction = instruction;
    }

    @Override
    public String getTargetLanguageInstruction() {
        return instruction;
    }

    @Override
    public boolean isConnectInstruction() {
        return false;
    }

    @Override
    public boolean isTargetCodeInstruction() {
        return true;
    }
}
