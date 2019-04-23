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
package de.monticore.lang.monticar.generator.cpp.viewmodel;

public final class AutopilotAdapterDataModel extends ViewModelBase {

    private String mainModelName;
    private String inputCount;
    private String outputCount;
    private String inputNames = "";
    private String outputNames = "";
    private String inputTypes = "";
    private String outputTypes = "";
    private String functionDeclarations = "";
    private String functionImplementations = "";

    public String getMainModelName() {
        return mainModelName;
    }
    public String getInputCount() {
        return inputCount;
    }
    public String getOutputCount() {
        return outputCount;
    }
    public String getInputNames() {
        return inputNames;
    }
    public String getOutputNames() {
        return outputNames;
    }
    public String getInputTypes() {
        return inputTypes;
    }
    public String getOutputTypes() {
        return outputTypes;
    }
    public String getFunctionDeclarations() {
        return functionDeclarations;
    }
    public String getFunctionImplementations() {
        return functionImplementations;
    }

    public void setMainModelName(String mainModelName) {
        this.mainModelName = mainModelName;
    }

    public void setInputCount(int count) {
        this.inputCount = Integer.toString(count);
    }
    public void setOutputCount(int count) {
        this.outputCount = Integer.toString(count);
    }

    public void addInput(String name, String type){
        //Add lines to name and type array
        this.inputNames += "\n    \"" + name + "\",";
        this.inputTypes += "\n    \"" + type + "\",";

        //Add line to function declarations and implementations
        String functionHeader = "EXPORT void set_input_" + name;
        if (type.equals("Q")){
            functionHeader += "(double v)";
            this.functionImplementations += functionHeader + " { AUTOPILOT_INSTANCE." + name + " = v; }\n";
        } else if (type.equals("Z")) {
            functionHeader += "(int v)";
            this.functionImplementations += functionHeader + " { AUTOPILOT_INSTANCE." + name + " = v; }\n";
        } else if (type.equals("CommonMatrixType")) {
            functionHeader += "(double *data, int size)";
            this.functionImplementations += functionHeader + " { copy_double_array_to_mat( data, size, AUTOPILOT_INSTANCE." + name + "); }\n";
        }
        this.functionDeclarations += functionHeader + ";\n";

    }

    public void addOutput(String name, String type){
        this.outputNames += "\n    \"" + name + "\",";
        this.outputTypes += "\n    \"" + type + "\",";

        String functionHeader = "";
        if (type.equals("Q")){
            functionHeader = "EXPORT double get_output_" + name + "()";
            this.functionImplementations += functionHeader + " { return AUTOPILOT_INSTANCE." + name + "; }\n";
        }
        this.functionDeclarations += functionHeader + ";\n";
    }
}
