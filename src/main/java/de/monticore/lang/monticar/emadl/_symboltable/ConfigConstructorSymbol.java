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
package de.monticore.lang.monticar.emadl._symboltable;

import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.emadl._ast.ASTNamedArgument;
import de.monticore.symboltable.CommonScopeSpanningSymbol;
import de.monticore.symboltable.Symbol;

import java.util.List;
import java.util.Optional;

public class ConfigConstructorSymbol extends CommonScopeSpanningSymbol {

    public static final ConfigConstructorKind KIND = new ConfigConstructorKind();

    private List<ASTNamedArgument> arguments;
    private ConfigurationSymbol configuration = null;

    public ConfigConstructorSymbol(String name) {
        super(name, KIND);
    }

    public List<ASTNamedArgument> getArguments() {
        return arguments;
    }

    public void setArguments(List<ASTNamedArgument> arguments) {
        this.arguments = arguments;
    }

    public ConfigurationSymbol getConfiguration() {
        if (configuration == null){
            //repeat because imports only seem to work after the first resolve for some reason
            getEnclosingScope().resolve(getName(), ConfigurationSymbol.KIND);
            Optional<Symbol> optSymbol = getEnclosingScope().resolve(getName(), ConfigurationSymbol.KIND);
            optSymbol.ifPresent(e -> setConfiguration((ConfigurationSymbol) e));
        }
        return configuration;
    }

    public void setConfiguration(ConfigurationSymbol configuration) {
        this.configuration = configuration;
    }

    public ConfigurationSymbol resolveConfiguration(){
        //todo: configuration.copy();
        ConfigurationSymbol config = getConfiguration();
        setArguments(config);
        return config;
    }

    private void setArguments(ConfigurationSymbol configuration){
        for (ASTNamedArgument argument : getArguments()){
            if (argument.getBooleanValue().isPresent()){
                configuration.setParameter(argument.getName(), argument.getBooleanValue().get());
            }
            else if (argument.getIntValue().isPresent()){
                configuration.setParameter(argument.getName(), argument.getIntValue().get());
            }
            else if (argument.getDoubleValue().isPresent()){
                configuration.setParameter(argument.getName(), argument.getDoubleValue().get());
            }
            else if (argument.getStringValue().isPresent()){
                configuration.setParameter(argument.getName(), argument.getStringValue().get());
            }
            else {
                throw new IllegalStateException("argument value has an unknown type: " + argument.getValue().getClass().getSimpleName());
            }
        }
    }
}
