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
package de.monticore.lang.monticar.generator.cmake;

import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;

import java.util.ArrayList;
import java.util.List;

/**
 * Representation of CMake Find files as Java class.
 * Additionally acts as view model for the corresponding freemarker template.
 *
 * @author Christoph Richter
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
