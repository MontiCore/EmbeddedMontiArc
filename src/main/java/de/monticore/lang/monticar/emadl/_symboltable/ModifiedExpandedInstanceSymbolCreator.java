/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbolCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.se_rwth.commons.logging.Log;

import java.util.List;
import java.util.Set;

public class ModifiedExpandedInstanceSymbolCreator extends EMAComponentInstanceSymbolCreator {

    @Override
    protected EMAComponentInstanceBuilder createInstance(EMAComponentSymbol cmp, Set<ResolvingFilter<? extends Symbol>> filters, List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols, String packageName) {
        ModifiedExpandedComponentInstanceBuilder builder = new ModifiedExpandedComponentInstanceBuilder();
        builder.setPackageName(packageName);
        //everything below is copied from super class
        builder.setSymbolReference(new EMAComponentSymbolReference(cmp.getName(), cmp.getEnclosingScope()))
                .addPorts(cmp.getPortsList())
                .addConnectors(cmp.getConnectors()).addResolutionDeclarationSymbols(cmp.getResolutionDeclarationSymbols()).addParameters(cmp.getParameters()).addArguments(cmp.getArguments());

        for (EMAConnectorSymbol connectorSymbol : cmp.getConnectors())
            Log.info(connectorSymbol.toString(), "Building Connector:");
        // add sub components
        for (EMAComponentInstantiationSymbol inst : cmp.getSubComponents()) {
            //      System.err.println("would create now: " + inst.getName() + "[" + inst.getComponentType().getFullName() + "]");
            Log.info(inst.getComponentType().getReferencedSymbol().howManyResolutionDeclarationSymbol() + "", "Important:");
            Log.debug(inst.toString(), "ComponentInstance CreateInstance PreSub");
            builder.addSubComponent(
                    createInstance(inst.getComponentType(), filters, inst.getComponentType().getReferencedSymbol().getResolutionDeclarationSymbols(), packageName)
                            .setName(inst.getName())
                            .addActualTypeArguments(inst.getComponentType().getFormalTypeParameters(),
                                    inst.getComponentType().getActualTypeArguments()).addResolvingFilters(filters).addResolutionDeclarationSymbols(inst.getComponentType().getResolutionDeclarationSymbols()).addParameters(inst.getComponentType().getReferencedSymbol().getParameters()).addArguments(inst.getComponentType().getReferencedSymbol().getArguments()).build());
            Log.debug(inst.getInstanceInformation().get().getInstanceNumberForArgumentIndex(0) + "", "InstanceInformation:");

            Log.debug(inst.toString(), "ComponentInstance CreateInstance PostSub");
        }

        // add inherited ports and sub components
        for (EMAComponentSymbol superCmp = cmp;
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
                    superCmp.getSuperComponent().get().getPortsList(),
                    superCmp.getSuperComponent().get().getFormalTypeParameters(),
                    superCmp.getSuperComponent().get().getActualTypeArguments());
            builder.addConnectorsIfNameDoesNotExists(superCmp.getSuperComponent().get().getConnectors());
            //Log.debug(superCmp.toString(), "superCmp pre lambda");
            superCmp.getSuperComponent().get().getSubComponents().stream().forEachOrdered(
                    inst -> builder.addSubComponentIfNameDoesNotExists(
                            createInstance(inst.getComponentType(), filters, null, packageName).setName(inst.getName())
                                    .addActualTypeArguments(inst.getComponentType().getFormalTypeParameters(),
                                            inst.getComponentType().getActualTypeArguments())
                                    .addResolvingFilters(filters).addResolutionDeclarationSymbols(inst.getComponentType().getReferencedSymbol().getResolutionDeclarationSymbols()).addParameters(inst.getComponentType().getReferencedSymbol().getParameters()).build())

            );
            //Log.debug(superCmp.toString(), "superCmp post lambda");

        }

        return builder;
    }

    @Override
    public void createInstances(EMAComponentSymbol topComponent, String instanceName) {
        super.createInstances(topComponent,instanceName.substring(0,1).toLowerCase() + instanceName.substring(1));
    }
}
