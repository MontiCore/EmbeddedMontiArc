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
package de.monticore.lang.monticar.generator.cpp.resolver;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.symboltable.Scope;

import java.util.Optional;

public class Resolver {

    private Scope symTab;

    public Resolver(Scope symTab) {
        this.symTab = symTab;
    }

    /*
    public Optional<ComponentSymbol> getComponentSymbol(String component) {
        return symTab.resolve(component, ComponentSymbol.KIND);
    }

    public Optional<PortSymbol> getPortSymbol(String port) {
        return symTab.resolve(port, PortSymbol.KIND);
    }

    public Optional<ConnectorSymbol> getConnectorSymbol(String con) {
        return symTab.resolve(con, ConnectorSymbol.KIND);
    }

    public Optional<ComponentInstanceSymbol> getComponentInstanceSymbol(String inst) {
        return symTab.resolve(inst, ComponentInstanceSymbol.KIND);
    }
*/
    public Optional<EMAComponentInstanceSymbol> getExpandedComponentInstanceSymbol(String inst) {
        return symTab.resolve(inst, EMAComponentInstanceSymbol.KIND);
    }
}
