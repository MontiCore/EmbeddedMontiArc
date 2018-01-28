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
package de.monticore.lang.monticar.emadl._cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortArraySymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.IODeclarationSymbol;
import de.monticore.lang.monticar.emadl._ast.ASTArchitectureConstructor;
import de.monticore.lang.monticar.emadl._symboltable.ArchPortConnectorSymbol;
import de.monticore.lang.monticar.emadl._symboltable.ArchitectureConstructorSymbol;
import de.monticore.lang.monticar.emadl.helper.ErrorCodes;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class CheckArchitectureIO implements EMADLASTArchitectureConstructorCoCo {

    @Override
    public void check(ASTArchitectureConstructor node) {
        ArchitectureConstructorSymbol constructor = (ArchitectureConstructorSymbol) node.getSymbol().get();

        List<IODeclarationSymbol> ioDeclarations = new ArrayList<>();
        ioDeclarations.addAll(constructor.getArchitecture().getInputs());
        ioDeclarations.addAll(constructor.getArchitecture().getOutputs());

        for (IODeclarationSymbol ioDeclaration : ioDeclarations){
            checkIODeclaration(constructor, ioDeclaration);
        }

        for (ArchPortConnectorSymbol port : constructor.getInputs()){
            checkIfInputPort(port);
            checkUnknownInput(constructor, port);
        }

        for (ArchPortConnectorSymbol port : constructor.getOutputs()){
            checkIfOutputPort(port);
            checkUnknownOutput(constructor, port);
        }
    }

    private void checkIODeclaration(ArchitectureConstructorSymbol constructor, IODeclarationSymbol ioDeclaration){
        if (ioDeclaration.isInput()){
            if (!constructor.getInputNames().contains(ioDeclaration.getName())){
                Log.error("0" + ErrorCodes.MISSING_IO_CODE + " " +
                                "Missing input with name '" + ioDeclaration.getName() + "'."
                        , constructor.getSourcePosition());
            }
        }
        else {
            if (!constructor.getOutputNames().contains(ioDeclaration.getName())){
                Log.error("0" + ErrorCodes.MISSING_IO_CODE + " " +
                                "Missing output with name '" + ioDeclaration.getName() + "'."
                        , constructor.getSourcePosition());
            }
        }
    }

    private void checkUnknownInput(ArchitectureConstructorSymbol constructor, ArchPortConnectorSymbol port){
        Optional<IODeclarationSymbol> input = constructor.getArchitecture().getInput(port.getAlias());
        if (!input.isPresent()){
            Log.error("0" + ErrorCodes.UNKNOWN_IO_CODE + " " +
                            "Input with name '" + port.getAlias() + "' does not exist. " +
                            "Existing inputs: " + Joiners.COMMA.join(constructor.getArchitecture().getOutputs()) + ". " +
                            "Tip: You can rename a port such that it fits to the input or output of the architecture in the following way: {portName as newName} -> Architecture() -> {outputPort}"
                    , port.getSourcePosition());
        }
    }

    private void checkUnknownOutput(ArchitectureConstructorSymbol constructor, ArchPortConnectorSymbol port){
        Optional<IODeclarationSymbol> output = constructor.getArchitecture().getOutput(port.getAlias());
        if (!output.isPresent()){
            Log.error("0" + ErrorCodes.UNKNOWN_IO_CODE + " " +
                            "Output with name '" + port.getAlias() + "' does not exist. " +
                            "Existing outputs: " + Joiners.COMMA.join(constructor.getArchitecture().getInputs()) + ". " +
                            "Tip: You can rename a port such that it fits to the input or output of the architecture in the following way: {portName as newName} -> Architecture() -> {outputPort}"
                    , port.getSourcePosition());
        }
    }

    private void checkIfInputPort(ArchPortConnectorSymbol port){
        Optional<PortArraySymbol> emaPort = port.getPort();
        if (emaPort.isPresent()){
            if (!emaPort.get().isIncoming()){
                Log.error("0" + ErrorCodes.INVALID_PORT_DIRECTION_CODE + " " +
                                "The Embedded Montiarc port '" + emaPort.get().getName() + "' is not an incoming port."
                        , port.getSourcePosition());
            }
        }
        else {
            portResolveError(port);
        }
    }

    private void checkIfOutputPort(ArchPortConnectorSymbol port){
        Optional<PortArraySymbol> emaPort = port.getPort();
        if (emaPort.isPresent()){
            if (emaPort.get().isIncoming()){
                Log.error("0" + ErrorCodes.INVALID_PORT_DIRECTION_CODE + " " +
                                "The Embedded Montiarc port '" + emaPort.get().getName() + "' is not an outgoing port."
                        , port.getSourcePosition());
            }
        }
        else {
            portResolveError(port);
        }
    }

    private void portResolveError(ArchPortConnectorSymbol port){
        Log.error("0" + ErrorCodes.UNKNOWN_EMA_PORT_CODE + " " +
                        "Embedded MontiArc Port '" + port.getName() + "' does not exist."
                , port.getSourcePosition());
    }

}
