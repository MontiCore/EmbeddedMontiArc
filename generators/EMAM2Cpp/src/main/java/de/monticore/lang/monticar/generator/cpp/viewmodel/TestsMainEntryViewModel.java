/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.viewmodel;

import java.util.Collections;
import java.util.List;

public final class TestsMainEntryViewModel extends ViewModelBase {

    private List<String> includes = Collections.emptyList();

    public List<String> getIncludes() {
        return includes;
    }

    public void setIncludes(List<String> includes) {
        this.includes = includes;
    }
}
