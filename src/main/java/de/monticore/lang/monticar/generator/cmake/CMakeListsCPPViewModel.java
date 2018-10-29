package de.monticore.lang.monticar.generator.cmake;

import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;

import java.util.ArrayList;
import java.util.HashSet;
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

    private List<String> cmakeCommandList = new ArrayList<>();

    private List<String> cmakeCommandListEnd = new ArrayList<>();

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

    public List<String> getCmakeCommandList() {
        return cmakeCommandList;
    }

    public void setCmakeCommandList(List<String> cmakeCommandList) {
        this.cmakeCommandList = cmakeCommandList;
    }

    public List<String> getCmakeCommandListEnd() {
        return cmakeCommandListEnd;
    }

    public void setCmakeCommandListEnd(List<String> cmakeCommandListEnd) {
        this.cmakeCommandListEnd = cmakeCommandListEnd;
    }
}
