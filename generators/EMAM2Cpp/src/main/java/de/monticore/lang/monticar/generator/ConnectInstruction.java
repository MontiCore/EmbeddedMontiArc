/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

/**
 */
public abstract class ConnectInstruction implements Instruction {
    Variable variable1, variable2;
    boolean useThis1 = false, useThis2 = false;

    public ConnectInstruction() {

    }

    public ConnectInstruction(Variable variable1, Variable variable2) {
        this.variable1 = variable1;
        this.variable2 = variable2;
    }


    public ConnectInstruction(Variable variable1, boolean useThis1, Variable variable2, boolean useThis2) {
        this.variable1 = variable1;
        this.variable2 = variable2;
        this.useThis1 = useThis1;
        this.useThis2 = useThis2;
    }

    public Variable getVariable1() {
        return variable1;
    }

    public Variable getVariable2() {
        return variable2;
    }

    public boolean isUseThis1() {
        return useThis1;
    }

    public boolean isUseThis2() {
        return useThis2;
    }

    public void setVariable1(Variable variable1) {
        this.variable1 = variable1;
    }

    public void setVariable2(Variable variable2) {
        this.variable2 = variable2;
    }

    public void setUseThis1(boolean useThis1) {
        this.useThis1 = useThis1;
    }

    public void setUseThis2(boolean useThis2) {
        this.useThis2 = useThis2;
    }

    @Override
    public boolean isConnectInstruction() {
        return true;
    }
}
