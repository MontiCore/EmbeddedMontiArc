/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class Method {

    String name;
    String typeName;
    List<Variable> parameters = new ArrayList<>();
    List<Instruction> instructions = new ArrayList<>();

    protected boolean isPublic = true;

    public Method() {

    }

    public Method(String name, String typeName) {
        this.name = name;
        this.typeName = typeName;
    }

    public Method(String name, String typeName, List<Variable> parameters) {
        this.name = name;
        this.typeName = typeName;
        this.parameters = parameters;
    }

    public void setName(String name) {
        this.name = name;
    }

    public void setReturnTypeName(String typeName) {
        this.typeName = typeName;
    }

    public void addParameter(Variable v) {
        parameters.add(v);
    }

    public void addParameter(Variable var, String variableName, String typeNameMontiCar,
                             String typeNameTargetLangauge, String includeName){
        var.setName(variableName);
        var.setVariableType(new VariableType(typeNameMontiCar, typeNameTargetLangauge, includeName));
        parameters.add(var);
    }

    public boolean addParameterUnique(Variable v) {
        boolean added = !containsParameter(v);
        if (added) {
            addParameter(v);
        }
        return added;
    }

    private boolean containsParameter(Variable v) {
        boolean found = false;
        for (Variable param : getParameters()) {
            found |= param.getName().contentEquals(v.getName());
        }
        return found;
    }

    public List<Variable> getParameters() {
        return parameters;
    }

    public String getReturnTypeName() {
        return typeName;
    }

    public String getName() {
        return name;
    }

    public List<Instruction> getInstructions() {
        return instructions;
    }

    public void addInstruction(Instruction instruction) {
        instructions.add(instruction);
    }

    public void addInstruction(int index, Instruction instruction) {
        instructions.add(index, instruction);
    }

    public void addInstructions(List<Instruction> instructions) {
        this.instructions.addAll(instructions);
    }

    public void setInstructions(List<Instruction> instructions) {
        this.instructions = instructions;
    }

    public String getTargetLanguageMethodCall() {
        String args = "";
        int size = getParameters().size();
        for (int i = 0; i < size - 1; i++) {
            args += String.format("%s, ", parameters.get(i).getNameTargetLanguageFormat());
        }
        args += parameters.get(size - 1).getNameTargetLanguageFormat();
        return String.format("%s(%s)", name, args);
    }

    public boolean isPublic() {
        return isPublic;
    }

    public void setPublic(boolean aPublic) {
        isPublic = aPublic;
    }
}
