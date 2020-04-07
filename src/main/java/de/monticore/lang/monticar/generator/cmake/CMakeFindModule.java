/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cmake;

import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;

import java.util.ArrayList;
import java.util.List;

/**
 * Representation of CMake Find files as Java class.
 * Additionally acts as view model for the corresponding freemarker template.
 *
 */
public class CMakeFindModule extends ViewModelBase {

    // fields
    private String packageName;
    private String includeName;
    private String libName;
    private List<String> includePaths = new ArrayList<>();
    private List<String> libPaths = new ArrayList<>();
    private Boolean findPath;
    private Boolean findLibrary;
    private Boolean fortranQuadMath = false;

    private boolean required;

    public CMakeFindModule(String moduleName, Boolean required) {
        this.packageName = moduleName;
        this.required = required;
        // guess rest
        this.includeName = moduleName.toLowerCase();
        this.libName = moduleName.toLowerCase();
        findPath = true;
        findLibrary = true;
    }

    public CMakeFindModule(String packageName, String includeName, String libName, List<String> includePaths, List<String> libPaths, Boolean findPath, Boolean findLibrary, boolean required) {
        this.packageName = packageName;
        this.includeName = includeName;
        this.libName = libName;
        this.includePaths = includePaths;
        this.libPaths = libPaths;
        this.findPath = findPath;
        this.findLibrary = findLibrary;
        this.required = required;
        if (packageName.equals("GFortran"))
            this.fortranQuadMath = true;
    }

    // methods

    public String getPackageName() {
        return packageName;
    }

    public void setPackageName(String packageName) {
        this.packageName = packageName;
    }

    public String getIncludeName() {
        return includeName;
    }

    public void setIncludeName(String includeName) {
        this.includeName = includeName;
    }

    public String getLibName() {
        return libName;
    }

    public void setLibName(String libName) {
        this.libName = libName;
    }

    public boolean isRequired() {
        return required;
    }

    public void setRequired(boolean required) {
        this.required = required;
    }

    public List<String> getIncludePaths() {
        return includePaths;
    }

    public void setIncludePaths(List<String> includePaths) {
        this.includePaths = includePaths;
    }

    public List<String> getLibPaths() {
        return libPaths;
    }

    public void setLibPaths(List<String> libPaths) {
        this.libPaths = libPaths;
    }

    public String getFileName() {
        return String.format("Find%s.cmake", packageName);
    }

    public Boolean getFindPath() {
        return findPath;
    }

    public void setFindPath(Boolean findPath) {
        this.findPath = findPath;
    }

    public Boolean getFindLibrary() {
        return findLibrary;
    }

    public void setFindLibrary(Boolean findLibrary) {
        this.findLibrary = findLibrary;
    }

    public Boolean getFortranQuadMath() {
        return fortranQuadMath;
    }

    public void setFortranQuadMath(Boolean fortranQuadMath) {
        this.fortranQuadMath = fortranQuadMath;
    }

}
