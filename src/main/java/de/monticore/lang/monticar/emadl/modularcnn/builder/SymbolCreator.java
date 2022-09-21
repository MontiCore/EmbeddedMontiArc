package de.monticore.lang.monticar.emadl.modularcnn.builder;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbolCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class SymbolCreator extends EMAComponentInstanceSymbolCreator {

    public SymbolCreator() {}

    @Override
    public void createInstances(EMAComponentSymbol topComponent, String instanceName){
        String realInstanceName = topComponent.getEnclosingScope().getLocalSymbols().keySet().stream().skip(topComponent.getEnclosingScope().getSymbolsSize()-1).findFirst().get();


        super.createInstances(topComponent,realInstanceName);
    }

    @Override
    protected EMAComponentInstanceBuilder createInstance(EMAComponentSymbol componentSymbol, Set<ResolvingFilter<? extends Symbol>> filters,
                                                         List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols, String packageName) {
       InstanceBuilder instanceBuilder = new InstanceBuilder();
       instanceBuilder.setPackageName(packageName);

       instanceBuilder.setSymbolReference(new EMAComponentSymbolReference(componentSymbol.getName(),componentSymbol.getEnclosingScope())).
                addPorts(componentSymbol.getPortsList())
               .addConnectors(componentSymbol.getConnectors())
               .addResolutionDeclarationSymbols(componentSymbol.getResolutionDeclarationSymbols())
               .addParameters(componentSymbol.getParameters())
               .addArguments(componentSymbol.getArguments());

       for (EMAConnectorSymbol connectorSymbol : componentSymbol.getConnectors()) {

       }

       for (EMAComponentInstantiationSymbol instance : componentSymbol.getSubComponents()){
           EMAComponentSymbolReference reference = instance.getComponentType();

           instanceBuilder.addSubComponent(
                   createInstance(instance.getComponentType(),filters,reference.getReferencedSymbol().getResolutionDeclarationSymbols(),packageName)
                           .setName(instance.getName())
                           .addActualTypeArguments(reference.getFormalTypeParameters(),reference.getActualTypeArguments())
                           .addResolvingFilters(filters)
                           .addResolutionDeclarationSymbols(reference.getResolutionDeclarationSymbols())
                           .addParameters(reference.getReferencedSymbol().getParameters())
                           .addArguments(reference.getReferencedSymbol().getArguments())
                           .build());

       }

       for (EMAComponentSymbol superCmp = componentSymbol; superCmp.getSuperComponent().isPresent(); superCmp = superCmp.getSuperComponent().get()) {
            EMAComponentSymbolReference superCmpRef = superCmp.getSuperComponent().get();
            if (superCmpRef.getFormalTypeParameters().size() != superCmpRef.getActualTypeArguments().size()){
                Log.error(String.format("Super component '%s' definition has %d generic parameters, but its"
                        + "instantiation has %d binds generic parameters -> MCNN SymbolCreator fail"));

                return null;
           }

            instanceBuilder.addPortsIfNameDoesNotExists(
                    superCmpRef.getPortsList(),
                    superCmpRef.getFormalTypeParameters(),
                    superCmpRef.getActualTypeArguments()
            );

            instanceBuilder.addConnectorsIfNameDoesNotExists(superCmpRef.getConnectors());

            superCmpRef.getSubComponents().stream().forEachOrdered(
                    instance -> instanceBuilder.addSubComponentIfNameDoesNotExists(
                            createInstance(instance.getComponentType(),filters,null,packageName)
                                    .setName(instance.getName())
                                    .addActualTypeArguments(instance.getComponentType().getFormalTypeParameters(),instance.getComponentType().getActualTypeArguments())
                                    .addResolvingFilters(filters)
                                    .addResolutionDeclarationSymbols(instance.getComponentType().getReferencedSymbol().getResolutionDeclarationSymbols())
                                    .addParameters(instance.getComponentType().getReferencedSymbol().getParameters())
                                    .build()
                    )
            );


       }

       return instanceBuilder;
    }
}
