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

    @Override
    public String getTargetLanguageInstruction() {
        String result = "";
        if (canBeThreaded) {
            //Log.error("yup");
            //this.threadName = "thread" + threadCounter;
            result += "std::thread "+ threadName + "( [ this ] {";
            //++threadCounter;
            result += "this->" + componentName + ".execute();});\n";

            return result;
        }

        if(dynamic){
            return String.format("if(true){ executeDynamicConnects(&%s); %s.execute();}\n",
                    componentName, componentName);
        }

        return componentName + ".execute();\n";
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
