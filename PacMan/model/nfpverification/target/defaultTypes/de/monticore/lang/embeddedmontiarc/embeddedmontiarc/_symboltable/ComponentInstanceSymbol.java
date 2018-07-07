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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import de.monticore.lang.monticar.ValueSymbol;
import de.monticore.lang.embeddedmontiarc.helper.SymbolPrinter;
import de.monticore.lang.montiarc.tagging._symboltable.TaggingScopeSpanningSymbol;
import de.monticore.symboltable.types.TypeSymbol;
import de.monticore.symboltable.types.references.TypeReference;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;


public class ComponentInstanceSymbol extends TaggingScopeSpanningSymbol implements ElementInstance {

    public static final EMAComponentInstanceKind KIND = EMAComponentInstanceKind.INSTANCE;

    public ComponentSymbolReference getComponentType(){};

    public Collection<ConnectorSymbol> getSimpleConnectors(){};

    public String getValue(){};

    public void setValue(String value) {};

    public List<ValueSymbol<TypeReference<TypeSymbol>>> getConfigArguments() {};

    public void addConfigArgument(ValueSymbol<TypeReference<TypeSymbol>> cfg) {};

    public void setConfigArgs(List<ValueSymbol<TypeReference<TypeSymbol>>> configArgs) {};

    public String toString() {};

    public Optional<InstanceInformation> getInstanceInformation(){};
}
