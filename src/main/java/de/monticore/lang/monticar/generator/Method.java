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

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
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
