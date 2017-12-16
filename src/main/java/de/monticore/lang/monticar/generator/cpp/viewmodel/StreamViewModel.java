package de.monticore.lang.monticar.generator.cpp.viewmodel;

import de.monticore.lang.monticar.generator.cpp.viewmodel.check.ComponentCheckViewModel;

import java.util.Collections;
import java.util.List;

public final class StreamViewModel extends ViewModelBase {

    private String name;
    private List<ComponentCheckViewModel> checks = Collections.emptyList();

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
}
