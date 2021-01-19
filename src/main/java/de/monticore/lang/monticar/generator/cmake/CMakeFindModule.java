/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cmake;

import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;

import java.util.*;

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
    private List<String> includePaths = new ArrayList();
    private List<String> libPaths = new ArrayList();
    private List<String> environmentVariableHints = new ArrayList();
    private List<String> includePathSuffixes = new ArrayList();
    private List<String> libraryPathSuffixes = new ArrayList();
    private Boolean findPath;
    private Boolean findLibrary;
    private Boolean fortranQuadMath = false;
    private Boolean findAsPackage = false;

    private boolean required;

    private final List<String> includeSuffixes = Arrays.asList("include");
    private final List<String> librarySuffixes = Arrays.asList("lib", "lib64", "lib/x86_64-linux-gnu", "examples/lib_win64", "build", "Release", "x64", "x86");

    public CMakeFindModule(String moduleName, Boolean required) {
        this.packageName = moduleName;
        this.required = required;
        includePathSuffixes.addAll(includeSuffixes);
        libraryPathSuffixes.addAll(librarySuffixes);
        // guess rest
        this.environmentVariableHints.add(String.format("%s_HOME", moduleName));
        this.includeName = moduleName.toLowerCase();
        this.libName = moduleName.toLowerCase();
        findPath = true;
        findLibrary = true;
    }

    public CMakeFindModule(String packageName, String includeName, String libName,
                           List<String> includePaths, List<String> libPaths,
                           List<String> additionalIncludePathSuffixes,
                           List<String> additionalLibraryPathSuffixes,
                           List<String> additionalEnvironmentVariableHints,
                           Boolean findPath, Boolean findLibrary, boolean required) {
        this.packageName = packageName;
        this.includeName = includeName;
        this.libName = libName;
        this.includePaths = includePaths;
        this.libPaths = libPaths;
        this.environmentVariableHints.add(String.format("%s_HOME", packageName));
        this.environmentVariableHints.addAll(additionalEnvironmentVariableHints);
        this.includePathSuffixes.addAll(includeSuffixes);
        this.libraryPathSuffixes.addAll(librarySuffixes);
        this.libraryPathSuffixes.addAll(additionalLibraryPathSuffixes);
        this.includePathSuffixes.addAll(additionalIncludePathSuffixes);
        this.findPath = findPath;
        this.findLibrary = findLibrary;
        this.required = required;
        if (packageName.equals("GFortran"))
            this.fortranQuadMath = true;
    }

    // methods

    public CMakeFindModule asFindAsPackage() {
        this.findLibrary = false;
        this.findPath = false;
        this.findAsPackage = true;
        return this;
    }

    // getters and setters

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

    public Boolean getFindAsPackage() {
        return findAsPackage;
    }

    public void setFindAsPackage(Boolean findAsPackage) {
        this.findAsPackage = findAsPackage;
    }

    public List<String> getIncludePathSuffixes() {
        return includePathSuffixes;
    }

    public void addAdditionalIncludePathSuffixes(Collection<String> additionalIncludePathSuffixes) {
        this.includePathSuffixes.addAll(additionalIncludePathSuffixes);
    }

    public List<String> getLibraryPathSuffixes() {
        return libraryPathSuffixes;
    }

    public void addAdditionalLibraryPathSuffixes(Collection<String> additionalLibraryPathSuffixes) {
        this.libraryPathSuffixes.addAll(additionalLibraryPathSuffixes);
    }

    public List<String> getEnvironmentVariableHints() {
        return environmentVariableHints;
    }

    public void addAdditionalEnvironmentVariableHints(List<String> additionalEnvironmentVariableHints) {
        this.environmentVariableHints.addAll(additionalEnvironmentVariableHints);
    }

    // equals and hashcode


    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof CMakeFindModule)) return false;
        CMakeFindModule that = (CMakeFindModule) o;
        return isRequired() == that.isRequired() &&
                Objects.equals(getPackageName(), that.getPackageName()) &&
                Objects.equals(getIncludeName(), that.getIncludeName()) &&
                Objects.equals(getLibName(), that.getLibName()) &&
                Objects.equals(getIncludePaths(), that.getIncludePaths()) &&
                Objects.equals(getLibPaths(), that.getLibPaths()) &&
                Objects.equals(getEnvironmentVariableHints(), that.getEnvironmentVariableHints()) &&
                Objects.equals(getIncludePathSuffixes(), that.getIncludePathSuffixes()) &&
                Objects.equals(getLibraryPathSuffixes(), that.getLibraryPathSuffixes()) &&
                Objects.equals(getFindPath(), that.getFindPath()) &&
                Objects.equals(getFindLibrary(), that.getFindLibrary()) &&
                Objects.equals(getFortranQuadMath(), that.getFortranQuadMath()) &&
                Objects.equals(getFindAsPackage(), that.getFindAsPackage());
    }

    @Override
    public int hashCode() {
        return Objects.hash(getPackageName(), getIncludeName(), getLibName(), getIncludePaths(), getLibPaths(), getEnvironmentVariableHints(), getIncludePathSuffixes(), getLibraryPathSuffixes(), getFindPath(), getFindLibrary(), getFortranQuadMath(), getFindAsPackage(), isRequired());
    }
}
