/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/**
 * Created by Michael von Wenckstern on 23.05.2016.
 *
 */
public class EMAComponentInstanceSymbolCreator {

    protected LinkedHashSet<EMAComponentSymbol> topComponents = new LinkedHashSet<>();

    public EMAComponentInstanceSymbolCreator() {
    }

    public static Scope getGlobalScope(final Scope scope) {
        Scope s = scope;
        while (s.getEnclosingScope().isPresent()) {
            s = s.getEnclosingScope().get();
        }
        return s;
    }

    /**
     * @param topComponent this is the scope where the top-level component is defined in
     */
    public void createInstances(EMAComponentSymbol topComponent, String instanceName) {
        MutableScope enclosingScope = (MutableScope) topComponent.getEnclosingScope();
        String fullInstanceName = Joiners.DOT.join(topComponent.getPackageName(), instanceName);

        if (enclosingScope.resolveDown(fullInstanceName, EMAComponentInstanceSymbol.KIND).isPresent()) {
            System.out.println("instances for top component + " + topComponent.getFullName() +
                    " is already created");
            Log.info("instances for top component + " + topComponent.getFullName() +
                            " is already created",
                    EMAComponentInstanceSymbolCreator.class.toString());
            return;
        }

        if (!topComponent.getFormalTypeParameters().isEmpty()) {
            Log.info("expanded component instance is not created, b/c top level has"
                            + " generic parameters and can, therefore, not be instantiated",
                    EMAComponentInstanceSymbolCreator.class.toString());
            return;
        }

        final Set<ResolvingFilter<? extends Symbol>> filters =
                topComponent.getSpannedScope().getResolvingFilters();


        EMAComponentInstanceBuilder builder =
                createInstance(topComponent, filters, null, fullInstanceName)
                        .setName(instanceName)
                        .setPackageName(topComponent.getPackageName());

        builder.fixSubComponentPackageNames();

        final EMAComponentInstanceSymbol instanceSymbol = builder.addResolvingFilters(filters).build();
        enclosingScope.add(instanceSymbol);
    }

    protected EMAComponentInstanceBuilder createInstance(EMAComponentSymbol cmp, final Set<ResolvingFilter<? extends Symbol>> filters, List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols, String packageName) {
        // TODO resolve generics and parameters
        //    System.err.println("create instance for: " + cmp.getName() + " [" + cmp.getFullName() + "]");
        EMAComponentInstanceBuilder builder =
                EMAComponentInstanceSymbol.builder()
                        .setSymbolReference(new EMAComponentSymbolReference(cmp.getName(),
                                cmp.getEnclosingScope()))
                        .addPorts(cmp.getPortsList())
                        .addConnectors(cmp.getConnectors())
                        .addResolutionDeclarationSymbols(cmp.getResolutionDeclarationSymbols())
                        .addParameters(cmp.getParameters())
                        .addArguments(cmp.getArguments())
                        .addPortInitials(cmp.getPortInitials())
                        .addComponentModifiers(cmp.getComponentModifiers());
        for (EMAConnectorSymbol emaConnectorSymbol : cmp.getConnectors())
            Log.info(emaConnectorSymbol.toString(), "Building Connector:");
        // add sub components
        for (EMAComponentInstantiationSymbol inst : cmp.getSubComponents()) {
            //      System.err.println("would create now: " + inst.getName() + "[" + inst.getComponentType().getFullName() + "]");
            Log.info(inst.getComponentType().getReferencedSymbol().howManyResolutionDeclarationSymbol() + "", "Important:");
            Log.debug(inst.toString(), "ComponentInstance CreateInstance PreSub");
            builder.addSubComponent(
                    createInstance(inst.getComponentType(), filters, inst.getComponentType().getReferencedSymbol().getResolutionDeclarationSymbols(), packageName + "." + inst.getName())
                            .setName(inst.getName()).setPackageName(packageName)
                            .addActualTypeArguments(inst.getComponentType().getFormalTypeParameters(),
                                    inst.getComponentType().getActualTypeArguments())
                            .addResolvingFilters(filters)
                            .addResolutionDeclarationSymbols(inst.getComponentType().getResolutionDeclarationSymbols())
                            .addParameters(inst.getComponentType().getReferencedSymbol().getParameters())
                            .addArguments(inst.getComponentType().getReferencedSymbol().getArguments())
                            .addPortInitials(inst.getComponentType().getReferencedSymbol().getPortInitials())
                            .addComponentModifiers(inst.getComponentType().getReferencedSymbol().getComponentModifiers())
                            .build());
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

}
