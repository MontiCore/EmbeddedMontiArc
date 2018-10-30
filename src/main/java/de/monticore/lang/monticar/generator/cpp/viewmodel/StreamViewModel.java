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
package de.monticore.lang.monticar.generator.cpp.viewmodel;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.cpp.viewmodel.check.ComponentCheckViewModel;
import de.monticore.lang.monticar.streamunits._symboltable.ComponentStreamUnitsSymbol;

import javax.sound.sampled.Port;
import java.util.ArrayList;
import java.util.Collection;
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
