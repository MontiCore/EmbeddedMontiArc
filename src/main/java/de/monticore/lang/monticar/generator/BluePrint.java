/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.optimization.MathInformationRegister;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class BluePrint {
    protected Generator generator;//currentGeneratorInstance
    protected List<Variable> variables = new ArrayList<>();
    protected List<Method> methods = new ArrayList<>();
    protected List<Variable> genericsVariableList = new ArrayList<>();
    MathInformationRegister mathInformationRegister = new MathInformationRegister(this);
    protected String name;
    protected String packageName;
    protected ExpandedComponentInstanceSymbol originalSymbol;

    public BluePrint(String name) {
        this.name = name;
    }

    public void setVariables(List<Variable> variables) {
        this.variables = variables;
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

    public List<Variable> getVariables() {
        return variables;
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

    public void addMethod(Method method) {
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

    public ExpandedComponentInstanceSymbol getOriginalSymbol() {
        return originalSymbol;
    }

    public void setOriginalSymbol(ExpandedComponentInstanceSymbol originalSymbol) {
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

    public void setGenerator(Generator generator) {
        this.generator = generator;
    }

    public MathCommandRegister getMathCommandRegister() {
        return generator.getMathCommandRegister();
    }
}
