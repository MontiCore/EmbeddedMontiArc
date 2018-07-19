package de.monticore.lang.monticar.generator.cmake;

import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;

import java.util.List;

/**
 * View model which is used by the freemarker template
 *
 * @author Christoph Richter
 */
public class CMakeListsCPPViewModel extends ViewModelBase {

    // const
    public static final String CMAKELISTS_FILE_NAME = "CMakeLists.txt";

    // fields
    private String compName;

    private List<CMakeFindModule> moduleDependencies;

    // methods

    public String getCompName() {
        return compName;
    }

    public void setCompName(String compName) {
        this.compName = compName;
    }

    public List<CMakeFindModule> getModuleDependencies() {
        return moduleDependencies;
    }

    public void setModuleDependencies(List<CMakeFindModule> moduleDependencies) {
        this.moduleDependencies = moduleDependencies;
    }
}
