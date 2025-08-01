/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.Variable;

import java.util.ArrayList;
import java.util.List;

public class VariableConstantArray extends Variable {

    protected List<String> values = new ArrayList<>();

    public VariableConstantArray(String name){
        super();
        this.setName(name);

    }

    @Override
    public boolean isConstantVariable() {
        return true;
    }

    @Override
    public String getConstantValue() {
        return String.join(", ", values);
    }

    public void generateInit(Method initMethod){
        for(int i = 0; i < this.getArraySize(); ++i){
            if(i < values.size()){
                initMethod.addInstruction(new TargetCodeInstruction(
                        String.format("%s[%d] = %s;\n", this.getName(), i, values.get(i))
                ));
            }
        }
    }

    public void addConstantInitValue(String value){
        values.add(value);
    }
}
