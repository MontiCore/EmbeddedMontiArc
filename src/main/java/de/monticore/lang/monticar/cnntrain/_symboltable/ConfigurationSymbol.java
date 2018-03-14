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

import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import java.util.*;

public class ConfigurationSymbol extends ConfigurationSymbolTOP {

    private Map<String, EntrySymbol> entryMap = new HashMap<>();
    private List<ConfigParameterSymbol> parameters = new ArrayList<>();

    public ConfigurationSymbol(String name) {
        super(name);
    }

    public Map<String, EntrySymbol> getEntryMap() {
        return entryMap;
    }

    public Collection<EntrySymbol> getEntries(){
        return getEntryMap().values();
    }

    public EntrySymbol getEntry(String name){
        return getEntryMap().get(name);
    }

    public List<ConfigParameterSymbol> getParameters() {
        return parameters;
    }

    public void setParameters(List<ConfigParameterSymbol> parameters) {
        this.parameters = parameters;
    }

    public Optional<ConfigParameterSymbol> getParameter(String name){
        for (ConfigParameterSymbol parameter : getParameters()){
            if (parameter.getName().equals(name)){
                return Optional.of(parameter);
            }
        }
        return Optional.empty();
    }

    private ConfigParameterSymbol getParameterOrError(String name){
        Optional<ConfigParameterSymbol> param = getParameter(name);
        if (param.isPresent()){
            return param.get();
        }
        else {
            throw new IllegalArgumentException("configuration parameter with name " + name + " does not exist.");
        }
    }

    public void setParameter(String name, Rational value){
        ConfigParameterSymbol parameter = getParameterOrError(name);
        if (value.getDivisor().intValue() == 1){
            parameter.setValue(value.getDividend().intValue());
        }
        else {
            parameter.setValue(value.doubleValue());
        }
    }

    public void setParameter(String name, boolean value){
        ConfigParameterSymbol parameter = getParameterOrError(name);
        parameter.setValue(value);
    }

    public void setParameter(String name, int value){
        ConfigParameterSymbol parameter = getParameterOrError(name);
        parameter.setValue(value);
    }

    public void setParameter(String name, double value){
        ConfigParameterSymbol parameter = getParameterOrError(name);
        parameter.setValue(value);
    }

    public void setParameter(String name, String value){
        ConfigParameterSymbol parameter = getParameterOrError(name);
        parameter.setValue(value);
    }
    
}
