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
package de.monticore.lang.monticar.cnntrain._symboltable;

import de.monticore.lang.monticar.cnntrain.annotations.Range;
import de.monticore.symboltable.SymbolKind;

import java.util.List;
import java.util.Map;

public abstract class NNArchitectureSymbol extends de.monticore.symboltable.CommonSymbol {
    public static final NNArchitectureSymbolKind KIND = NNArchitectureSymbolKind.INSTANCE;

    public NNArchitectureSymbol(String name) {
        super(name, KIND);
    }

    abstract public List<String> getInputs();
    abstract public List<String> getOutputs();
    abstract public Map<String, List<Integer>> getDimensions();
    abstract public Map<String, Range> getRanges();
    abstract public Map<String, String> getTypes();
}