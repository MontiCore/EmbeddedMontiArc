package de.monticore.lang.monticar.generator.roscpp.util;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.roscpp.helper.PrinterHelper;
import de.monticore.lang.monticar.generator.rosmsg.util.FileContent;

import java.util.HashSet;
import java.util.Set;

public class AdapterBluePrint extends BluePrintCPP {
    private Set<String> variableNames = new HashSet<>();

    private Method constructor;
    private Method init;
    private Method tick;
    private Variable componentField;


    @Override
    public void addVariable(Variable variable) {
        if (!variableNames.contains(variable.getName())) {
            variableNames.add(variable.getName());
            super.addVariable(variable);
        }
    }

    public AdapterBluePrint(String name, String componentName) {
        super(name);

        addConstructor(name);
        addComponentField(componentName);

        addInit(componentName);

        tick = new Method("tick", "void");
        this.addMethod(tick);
    }

    private void addInit(String componentName) {
        Variable compParam = new Variable();
        compParam.setTypeNameTargetLanguage(componentName + "*");
        compParam.setName("comp");
        init = new Method("init", "void");
        init.addParameter(compParam);
        this.addMethod(init);
    }

    private void addComponentField(String componentName) {
        componentField = new Variable();
        componentField.setTypeNameTargetLanguage(componentName + "*");
        componentField.setName("component");
        this.addVariable(componentField);
    }

    private void addConstructor(String name) {
        constructor = new Method();
        constructor.setName(name);
        constructor.setReturnTypeName("");
        constructor.addInstruction(new TargetCodeInstruction(""));
        this.addMethod(constructor);
    }


    public Method getConstructor() {
        return constructor;
    }

    public Method getInit() {
        return init;
    }

    public Method getTick() {
        return tick;
    }

    public Variable getComponentField() {
        return componentField;
    }

    public void getAdapterFileContent(EMAComponentInstanceSymbol component, FileContent apdapter) {
        apdapter.setFileName(getName() + ".h");
        apdapter.setFileContent(PrinterHelper.printClass(this, ": public IAdapter_" + NameHelper.getComponentNameTargetLanguage(component.getFullName())));
    }
}
