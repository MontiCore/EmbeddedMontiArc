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

import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter;

/**
 * @author Sascha Schneiders
 */
public class ExecuteInstruction implements Instruction {
    String componentName;
    BluePrint bluePrint;
    String threadName = null;
    boolean canBeThreaded = false;
    boolean dynamic = false;
    public static int threadCounter = 0;

    public ExecuteInstruction(String componentName, BluePrint bluePrint, boolean canBeThreaded) {
        this.bluePrint = bluePrint;
        this.canBeThreaded = canBeThreaded;
        while (!bluePrint.getVariable(componentName).isPresent() && componentName.contains("_")) {
            componentName = componentName.replaceFirst("\\_", "[");
            componentName = componentName.replaceFirst("\\_", "]");
        }
        this.componentName = GeneralHelperMethods.getTargetLanguageVariableInstanceName(componentName);
        if (canBeThreaded)
            this.threadName = "thread" + ++threadCounter;
    }

    public String getComponentName() {
        return componentName;
    }

    public void setComponentName(String componentName) {
        this.componentName = componentName;
    }

    public boolean isCanBeThreaded() {
        return canBeThreaded;
    }

    public void setCanBeThreaded(boolean canBeThreaded) {
        this.canBeThreaded = canBeThreaded;
    }

    public String getThreadName() {
        return threadName;
    }

    protected String addConditionIfDynamic(String exec){

        if(isDynamic()){


            String inst = componentName.substring(0, componentName.indexOf("["));
            String id = componentName.substring(componentName.indexOf("[")+1, componentName.lastIndexOf("]"));

            return String.format("if(__%s_connected[%s]){ executeDynamicConnects(&(%s)); %s}", inst, id,componentName, exec);
        }

        return exec;
    }

    @Override
    public String getTargetLanguageInstruction() {
        String result = "";
        if (canBeThreaded) {
            //Log.error("yup");
            //this.threadName = "thread" + threadCounter;
            result += "std::thread "+ threadName + "( [ this ] {";
            //++threadCounter;

            //OLD: result += "this->" + componentName + ".execute();});\n";
            result += addConditionIfDynamic("this->"+componentName+".execute();");
            result += "});\n";

            return result;
        }

        return addConditionIfDynamic(componentName + ".execute();")+"\n";
    }

    @Override
    public boolean isConnectInstruction() {
        return false;
    }

    @Override
    public boolean isExecuteInstruction() {
        return true;
    }

    public boolean isDynamic() {
        return dynamic;
    }

    public void setDynamic(boolean dynamic) {
        this.dynamic = dynamic;
    }
}
