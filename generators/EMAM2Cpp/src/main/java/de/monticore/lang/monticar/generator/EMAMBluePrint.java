/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.optimization.MathInformationRegister;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

/**
 */
public class EMAMBluePrint {
    protected EMAMGenerator generator;//currentGeneratorInstance
    protected List<Variable> variables = new ArrayList<>();
    protected List<Method> methods = new ArrayList<>();
    protected List<Variable> genericsVariableList = new ArrayList<>();
    MathInformationRegister mathInformationRegister = new MathInformationRegister(this);
    protected String name;
    protected String packageName;
    protected EMAComponentInstanceSymbol originalSymbol;

    public EMAMBluePrint(String name) {
        this.name = name;
    }

    public void setVariables(List<Variable> variables) {
        this.variables = variables;
    }

    public List<Variable> getVariables() {
        return variables;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public void setPackageName(String name) {
        this.packageName = name;
    }

    public String getPackageName() {
        return packageName;
    }

    public int howManyVariables() {
        return variables.size();
    }

    public Optional<Variable> getVariable(String variableName) {
        for (Variable v : variables)
            if (v.getName().equals(variableName))
                return Optional.of(v);
        return Optional.empty();
    }

    public void addVariable(Variable v) {
        variables.add(v);
    }

    public void removeVariable(Variable v) {
        variables.remove(v);
    }

    public void replaceVariable(Variable varOld, Variable varNew){
        Collections.replaceAll(variables,varOld, varNew);
    }
    public void addMethod(Method method) {
        for(Method meth: methods){
            if(meth.getName().equals(method.getName())){
                return;
            }
        }
        methods.add(method);
    }

    public List<Method> getMethods() {
        return methods;
    }

    public MathInformationRegister getMathInformationRegister() {
        return mathInformationRegister;
    }

    public void setMathInformationRegister(MathInformationRegister mathInformationRegister) {
        this.mathInformationRegister = mathInformationRegister;
    }

    public void addGenericVariable(Variable genericVariable) {
        genericsVariableList.add(genericVariable);
    }

    public List<Variable> getGenericsVariableList() {
        return genericsVariableList;
    }

    public EMAComponentInstanceSymbol getOriginalSymbol() {
        return originalSymbol;
    }

    public void setOriginalSymbol(EMAComponentInstanceSymbol originalSymbol) {
        this.originalSymbol = originalSymbol;
    }

    public Optional<Method> getMethod(String name) {
        for (Method method : getMethods()) {
            if (method.getName().equals(name))
                return Optional.of(method);
        }
        return Optional.empty();
    }

    public Generator getGenerator() {
        return generator;
    }

    public void setGenerator(EMAMGenerator generator) {
        this.generator = generator;
    }

    public MathCommandRegister getMathCommandRegister() {
        return generator.getMathCommandRegister();
    }
}
