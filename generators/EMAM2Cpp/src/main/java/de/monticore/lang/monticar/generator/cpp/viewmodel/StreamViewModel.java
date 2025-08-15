/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.viewmodel;

import de.monticore.lang.monticar.generator.cpp.viewmodel.check.ComponentCheckViewModel;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class StreamViewModel extends ViewModelBase {

    private String name;
    private List<ComponentCheckViewModel> checks = Collections.emptyList();
    public List<String> outputPortNames = new ArrayList<>();

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public List<ComponentCheckViewModel> getChecks() {
        return checks;
    }

    public void setChecks(List<ComponentCheckViewModel> checks) {
        this.checks = checks;
    }

    public List<String> getOutputPortNames() {
        return outputPortNames;
    }

    public void setOutputPortNames(List<String> outputPortNames) {
        this.outputPortNames = outputPortNames;
    }
}
