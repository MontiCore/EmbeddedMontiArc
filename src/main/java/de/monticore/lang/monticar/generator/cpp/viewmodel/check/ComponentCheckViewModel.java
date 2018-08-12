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
package de.monticore.lang.monticar.generator.cpp.viewmodel.check;

import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;

import java.util.Collections;
import java.util.Map;

public final class ComponentCheckViewModel extends ViewModelBase {

    private Map<String, String> inputPortName2Value = Collections.emptyMap();
    private Map<String, IOutputPortCheck> outputPortName2Check = Collections.emptyMap();

    public Map<String, String> getInputPortName2Value() {
        return inputPortName2Value;
    }

    public void setInputPortName2Value(Map<String, String> inputPortName2Value) {
        this.inputPortName2Value = inputPortName2Value;
    }

    public Map<String, IOutputPortCheck> getOutputPortName2Check() {
        return outputPortName2Check;
    }

    public void setOutputPortName2Check(Map<String, IOutputPortCheck> outputPortName2Check) {
        this.outputPortName2Check = outputPortName2Check;
    }
}
