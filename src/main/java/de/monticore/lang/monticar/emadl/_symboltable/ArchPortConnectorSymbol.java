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

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortArraySymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.IODeclarationSymbol;
import de.monticore.symboltable.CommonSymbol;

import java.util.List;
import java.util.Optional;

public class ArchPortConnectorSymbol extends CommonSymbol {

    public static final ArchPortConnectorKind KIND = new ArchPortConnectorKind();

    private String alias = null;
    private PortArraySymbol port = null;

    private IODeclarationSymbol ioDeclaration = null;

    public ArchPortConnectorSymbol(String name) {
        super(name, KIND);
        setAlias(name);
    }

    public String getAlias() {
        if (alias == null){
            alias = getName();
        }
        return alias;
    }

    public void setAlias(String alias) {
        this.alias = alias;
    }

    public Optional<IODeclarationSymbol> getIoDeclaration() {
        return Optional.ofNullable(ioDeclaration);
    }

    public void setIoDeclaration(IODeclarationSymbol ioDeclaration) {
        this.ioDeclaration = ioDeclaration;
    }

    public Optional<PortArraySymbol> getPort() {
        if (port == null){
            getEnclosingScope().resolve(getName(), PortArraySymbol.KIND)
                    .ifPresent(e -> setPort((PortArraySymbol) e));
        }
        return Optional.ofNullable(port);
    }

    public void setPort(PortArraySymbol port) {
        this.port = port;
    }

    public void linkIODeclaration(ArchitectureSymbol architecture){
        if (getPort().get().isIncoming()){
            linkIODeclaration(architecture.getInputs());
        }
        else {
            linkIODeclaration(architecture.getOutputs());
        }
    }

    public void linkIODeclaration(List<IODeclarationSymbol> ioDeclarations){
        for (IODeclarationSymbol ioDeclaration : ioDeclarations){
            if (getAlias().equals(ioDeclaration.getName())){
                setIoDeclaration(ioDeclaration);
            }
        }
    }
}
