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
import java.util.LinkedHashSet;
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

    private LinkedHashSet<CMakeFindModule> moduleDependencies;

    private List<String> cmakeCommandList = new ArrayList<>();

    private List<String> cmakeCommandListEnd = new ArrayList<>();

    // methods

    public String getCompName() {
        return compName;
    }

    public void setCompName(String compName) {
        this.compName = compName;
    }

    public LinkedHashSet<CMakeFindModule> getModuleDependencies() {
        return moduleDependencies;
    }

    public void setModuleDependencies(LinkedHashSet<CMakeFindModule> moduleDependencies) {
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
