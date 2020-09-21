/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.*;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.builder.EMADynamicEventHandlerInstanceBuilder;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.types.TypeSymbol;
import de.monticore.symboltable.types.references.ActualTypeArgument;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

public class EMADynamicComponentInstanceBuilder extends EMAComponentInstanceBuilder {

    protected List<EMADynamicEventHandlerSymbol> eventHandler = new ArrayList<>();
    protected boolean dynamicInstance = false;

    public EMADynamicComponentInstanceBuilder addEventHandler(EMADynamicEventHandlerSymbol eh) {
        this.eventHandler.add(eh);
        return this;
    }

    public EMADynamicComponentInstanceBuilder addEventHandlers(EMADynamicEventHandlerSymbol... eh) {
        for (EMADynamicEventHandlerSymbol p : eh) {
            this.addEventHandler(p);
        }
        return this;
    }

    public EMADynamicComponentInstanceBuilder addEventHandlers(Collection<EMADynamicEventHandlerSymbol> eh) {
        eh.stream().forEachOrdered(p -> this.addEventHandler(p));
        return this;
    }

    public EMADynamicComponentInstanceBuilder addDynamicInstance(boolean dynamicInstance) {
        this.dynamicInstance = dynamicInstance;
        return this;
    }


    @Override
    protected EMAComponentInstanceSymbol instantiateComponentSymbol() {
        return new EMADynamicComponentInstanceSymbol(this.name.get(), this.symbolReference.get());
    }

    @Override
    protected void instantiateConnectorSymbol(EMAConnectorSymbol c, String fullName,
            MutableScope scope) {
        scope.add(EMADynamicConnectorInstanceSymbol.newAndInstantiate(c, fullName));
    }

    @Override
    protected void instantiatePortSymbol(EMAPortSymbol port, String packageName, MutableScope scope) {
        if (port instanceof EMADynamicPortArraySymbol) {
            //port array
            instantiateDynamicPortArraySymbol((EMADynamicPortArraySymbol) port, packageName, scope);
        } else {
            scope.add(EMADynamicPortInstanceSymbol.newAndInstantiate(port, packageName));
        }
    }

    protected void instantiateDynamicPortArraySymbol(EMADynamicPortArraySymbol sym, String packageName,
            MutableScope scope) {
        for (int i = 0; i < sym.getDimension(); ++i) {
            EMADynamicPortInstanceSymbol inst = new EMADynamicPortInstanceSymbol(sym.getName() + "[" + (i + 1) + "]");
            inst.instantiate(sym, packageName);
            if (i < sym.getNonDynamicDimension()) {
                inst.setDynamic(false);
            }
            scope.add(inst);
        }
        if (sym.isDimensionInfinite()) {
            EMADynamicPortInstanceSymbol inst = new EMADynamicPortInstanceSymbol(sym.getName() + "[oo]");
            inst.instantiate(sym, packageName);
            inst.setDimensionInfinite(true);
            scope.add(inst);
        }
    }

    @Override
    protected void addOtherToComponentInstance(EMAComponentInstanceSymbol sym) {
        super.addOtherToComponentInstance(sym);
        final MutableScope scope = (MutableScope) sym.getSpannedScope();
        if (sym instanceof EMADynamicComponentInstanceSymbol) {
            eventHandler.stream().forEach(e -> {
                scope.add(EMADynamicEventHandlerInstanceBuilder.getINSTANCE().build(e));

//                Collection<EMADynamicPortArraySymbol> connPorts = e.getConnectPorts();
//                Optional<EMADynamicPortArraySymbol> pA = Optional.empty();
//                if(connPorts.size() > 0){
//                    pA = Optional.of(connPorts.iterator().next());
//
//                }
//                for(EMADynamicComponentInstantiationSymbol sub : e.getDynamicSubComponents()){
//                    sub.setDynamic(true);
//                    sub.setArray(true);
//                    sub.setNonDynamicDimension(0);
//                    sub.setDimension(0);
//                    if(pA.isPresent()){
//                        sub.setDimension(pA.get().getDimension());
////                        if(pA.get().getNameNonDynamicSizeDependsOn().isPresent()){
//                            sub.setDimensionDependsOn(pA.get().getNameSizeDependsOn());
////                        }
//                    }
////                    subComponents.add(sub);
//                }
            });
            ((EMADynamicComponentInstanceSymbol) sym).setDynamicInstance(this.dynamicInstance);
            // Log.error("TODO: EMADynamicComponentInstanceBuilder.build -> event handler, event condition expansion + resolving + generics + ports");
        }
    }

}
