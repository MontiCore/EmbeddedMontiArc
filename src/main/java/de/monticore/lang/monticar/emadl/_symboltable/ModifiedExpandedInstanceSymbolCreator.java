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

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.*;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.se_rwth.commons.logging.Log;

import java.util.List;
import java.util.Set;

public class ModifiedExpandedInstanceSymbolCreator extends EmbeddedMontiArcExpandedComponentInstanceSymbolCreator {

    @Override
    protected ExpandedComponentInstanceBuilder createInstance(ComponentSymbol cmp, Set<ResolvingFilter<? extends Symbol>> filters, List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols) {
        ModifiedExpandedComponentInstanceBuilder builder = new ModifiedExpandedComponentInstanceBuilder();

        //everything below is copied from super class
        builder.setSymbolReference(new ComponentSymbolReference(cmp.getName(), cmp.getEnclosingScope()))
                .addPorts(cmp.getPorts())
                .addConnectors(cmp.getConnectors()).addResolutionDeclarationSymbols(cmp.getResolutionDeclarationSymbols()).addParameters(cmp.getParameters()).addArguments(cmp.getArguments());

        for (ConnectorSymbol connectorSymbol : cmp.getConnectors())
            Log.info(connectorSymbol.toString(), "Building Connector:");
        // add sub components
        for (ComponentInstanceSymbol inst : cmp.getSubComponents()) {
            //      System.err.println("would create now: " + inst.getName() + "[" + inst.getComponentType().getFullName() + "]");
            Log.info(inst.getComponentType().getReferencedSymbol().howManyResolutionDeclarationSymbol() + "", "Important:");
            Log.debug(inst.toString(), "ComponentInstance CreateInstance PreSub");
            builder.addSubComponent(
                    createInstance(inst.getComponentType(), filters, inst.getComponentType().getReferencedSymbol().getResolutionDeclarationSymbols())
                            .setName(inst.getName())
                            .addActualTypeArguments(inst.getComponentType().getFormalTypeParameters(),
                                    inst.getComponentType().getActualTypeArguments()).addResolvingFilters(filters).addResolutionDeclarationSymbols(inst.getComponentType().getResolutionDeclarationSymbols()).addParameters(inst.getComponentType().getReferencedSymbol().getParameters()).addArguments(inst.getComponentType().getReferencedSymbol().getArguments()).build());
            Log.debug(inst.getInstanceInformation().get().getInstanceNumberForArgumentIndex(0) + "", "InstanceInformation:");

            Log.debug(inst.toString(), "ComponentInstance CreateInstance PostSub");
        }

        // add inherited ports and sub components
        for (ComponentSymbol superCmp = cmp;
             superCmp.getSuperComponent().isPresent();
             superCmp = superCmp.getSuperComponent().get()) {

            if (superCmp.getSuperComponent().get().getFormalTypeParameters().size() !=
                    superCmp.getSuperComponent().get().getActualTypeArguments().size()) {
                Log.error(String.format("Super component '%s' definition has %d generic parameters, but its"
                                + "instantiation has %d binds generic parameters", superCmp.getFullName(),
                        superCmp.getSuperComponent().get().getFormalTypeParameters().size(),
                        superCmp.getSuperComponent().get().getActualTypeArguments().size()));
                return null;
            }

            builder.addPortsIfNameDoesNotExists(
                    superCmp.getSuperComponent().get().getPorts(),
                    superCmp.getSuperComponent().get().getFormalTypeParameters(),
                    superCmp.getSuperComponent().get().getActualTypeArguments());
            builder.addConnectorsIfNameDoesNotExists(superCmp.getSuperComponent().get().getConnectors());
            //Log.debug(superCmp.toString(), "superCmp pre lambda");
            superCmp.getSuperComponent().get().getSubComponents().stream().forEachOrdered(
                    inst -> builder.addSubComponentIfNameDoesNotExists(
                            createInstance(inst.getComponentType(), filters, null).setName(inst.getName())
                                    .addActualTypeArguments(inst.getComponentType().getFormalTypeParameters(),
                                            inst.getComponentType().getActualTypeArguments())
                                    .addResolvingFilters(filters).addResolutionDeclarationSymbols(inst.getComponentType().getReferencedSymbol().getResolutionDeclarationSymbols()).addParameters(inst.getComponentType().getReferencedSymbol().getParameters()).build())

            );
            //Log.debug(superCmp.toString(), "superCmp post lambda");

        }

        return builder;
    }

    @Override
    public void createInstances(ComponentSymbol topComponent) {
        super.createInstances(topComponent);
    }
}
