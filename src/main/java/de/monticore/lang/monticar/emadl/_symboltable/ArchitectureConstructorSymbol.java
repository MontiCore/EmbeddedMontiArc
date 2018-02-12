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

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.emadl._ast.ASTNamedArgument;
import de.monticore.symboltable.CommonScopeSpanningSymbol;
import de.monticore.symboltable.Symbol;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;


public class ArchitectureConstructorSymbol extends CommonScopeSpanningSymbol {

    public static final ArchitectureConstructorKind KIND = new ArchitectureConstructorKind();

    private List<ASTNamedArgument> arguments;
    private List<ArchPortConnectorSymbol> inputs;
    private List<ArchPortConnectorSymbol> outputs;
    private ArchitectureSymbol architecture = null;

    public ArchitectureConstructorSymbol(String name) {
        super(name, KIND);
    }

    public List<ASTNamedArgument> getArguments() {
        return arguments;
    }

    public void setArguments(List<ASTNamedArgument> arguments) {
        this.arguments = arguments;
    }

    public List<ArchPortConnectorSymbol> getInputs() {
        return inputs;
    }

    public void setInputs(List<ArchPortConnectorSymbol> inputs) {
        this.inputs = inputs;
    }

    public List<ArchPortConnectorSymbol> getOutputs() {
        return outputs;
    }

    public void setOutputs(List<ArchPortConnectorSymbol> outputs) {
        this.outputs = outputs;
    }

    public ArchitectureSymbol getArchitecture() {
        if (architecture == null){
            //repeat because imports only seem to work after the first resolve for some reason
            getEnclosingScope().resolve(getName(), ArchitectureSymbol.KIND);
            Optional<Symbol> optSymbol = getEnclosingScope().resolve(getName(), ArchitectureSymbol.KIND);
            optSymbol.ifPresent(e -> setArchitecture((ArchitectureSymbol) e));
        }
        return architecture;
    }

    public void setArchitecture(ArchitectureSymbol architecture) {
        this.architecture = architecture;
    }

    public List<String> getInputNames(){
        List<String> names = new ArrayList<>();
        for (ArchPortConnectorSymbol port : getInputs()){
            names.add(port.getAlias());
        }
        return names;
    }

    public List<String> getOutputNames(){
        List<String> names = new ArrayList<>();
        for (ArchPortConnectorSymbol port : getOutputs()){
            names.add(port.getAlias());
        }
        return names;
    }

    public ArchitectureSymbol resolveArchitecture(){
        //todo: architecture.copy();
        ArchitectureSymbol architecture = getArchitecture();
        setArguments(architecture);
        linkIOPorts(architecture);

        architecture.resolveOrError();
        return architecture;
    }

    private void linkIOPorts(ArchitectureSymbol architecture) {
        for (ArchPortConnectorSymbol port : getInputs()){
            port.linkIODeclaration(architecture);
        }
        for (ArchPortConnectorSymbol port : getOutputs()){
            port.linkIODeclaration(architecture);
        }
    }

    private void setArguments(ArchitectureSymbol architecture){
        for (ASTNamedArgument argument : getArguments()){
            if (argument.getBooleanValue().isPresent()){
                architecture.setParameter(argument.getName(), argument.getBooleanValue().get());
            }
            else if (argument.getIntValue().isPresent()){
                architecture.setParameter(argument.getName(), argument.getIntValue().get());
            }
            else if (argument.getDoubleValue().isPresent()){
                architecture.setParameter(argument.getName(), argument.getDoubleValue().get());
            }
            else if (argument.getStringValue().isPresent()){
                architecture.setParameter(argument.getName(), argument.getStringValue().get());
            }
            else {
                throw new IllegalStateException("argument value has an unknown type: " + argument.getValue().getClass().getSimpleName());
            }
        }
    }


}
