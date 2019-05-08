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

import java.util.HashMap;
import java.util.Map;

/**
 *
 */
public class MultiParamValueSymbol extends ValueSymbol {
    public static final MultiParamValueSymbolKind KIND = new MultiParamValueSymbolKind();

    private Map<String, Object> parameters;

    public MultiParamValueSymbol() {
        super("", KIND);
        this.parameters = new HashMap<>();
    }

    public Map<String, Object> getParameters() {
        return parameters;
    }

    public Object getParameter(final String parameterName) {
        return parameters.get(parameterName);
    }

    public void addParameter(final String parameterName, final Object value) {
        parameters.put(parameterName, value);
    }

    @Override
    public String toString() {
        return super.toString() + '{' + parameters + '}';
    }
}