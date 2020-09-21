/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbolCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.*;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class EMADynamicComponentInstanceSymbolCreator extends EMAComponentInstanceSymbolCreator {



//    protected EMAComponentInstanceBuilder createInstance(EMAComponentSymbol cmp, Set<ResolvingFilter<? extends Symbol>> filters, List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols, String packageName) {
//
//
//        //TODO Change order of build: EMADynamicComponentSymbolReference does not extends EMADynamicComponentSymbol but it should be because of its arguments
//        // no arguments passed to create instance
//
//        if(cmp instanceof  EMADynamicComponentSymbolReference){
//            return this.createInstance(((EMADynamicComponentSymbolReference) cmp).getReferencedSymbol(), filters,resolutionDeclarationSymbols,packageName);
//        }
////        if (cmp instanceof EMADynamicComponentSymbol) {
//        return this.createInstance((EMADynamicComponentSymbol) cmp, filters, resolutionDeclarationSymbols, packageName);
////        }
//
////        return super.createInstance(cmp, filters, resolutionDeclarationSymbols, packageName);
//    }

    @Override
    protected EMADynamicComponentInstanceBuilder createInstance(EMAComponentSymbol cmp, Set<ResolvingFilter<? extends Symbol>> filters, List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols, String packageName) {

        EMADynamicComponentInstanceBuilder builder = this.getNewBuilder();

        builder.setSymbolReference(new EMADynamicComponentSymbolReference(cmp.getName(), cmp.getEnclosingScope()))
                .addPorts(cmp.getPortsList())
                .addConnectors(cmp.getConnectors())
                .addResolutionDeclarationSymbols(cmp.getResolutionDeclarationSymbols())
                .addParameters(cmp.getParameters())
                .addArguments(cmp.getArguments());

        Collection<EMADynamicEventHandlerSymbol> eventHandlers = new ArrayList<>();

        if(cmp instanceof EMADynamicComponentSymbol) {
//            builder.addEventHandlers(((EMADynamicComponentSymbol)cmp).getEventHandlers());
            eventHandlers = ((EMADynamicComponentSymbol)cmp).getEventHandlers();

        }else if(cmp instanceof EMADynamicComponentSymbolReference){
//            builder.addEventHandlers(((EMADynamicComponentSymbolReference)cmp).getEventHandlers());
            eventHandlers = ((EMADynamicComponentSymbolReference)cmp).getEventHandlers();
        }

        builder.addEventHandlers(eventHandlers);
        for(EMADynamicEventHandlerSymbol eve : eventHandlers){
            Collection<EMADynamicPortArraySymbol> connPorts = eve.getConnectPorts();
            Optional<EMADynamicPortArraySymbol> pA = Optional.empty();
            if(connPorts.size() > 0){
                pA = Optional.of(connPorts.iterator().next());
            }
            for(EMADynamicComponentInstantiationSymbol inst : eve.getDynamicSubComponents()){
                inst.setDynamic(true);
                inst.setArray(true);
                inst.setNonDynamicDimension(0);
                inst.setDimension(0);
                if(pA.isPresent()){
                    inst.setDimension(pA.get().getDimension());
                    inst.setDimensionDependsOn(pA.get().getNameSizeDependsOn());
                }
                addSubComponents(inst, builder, filters, resolutionDeclarationSymbols, packageName);
            }
        }


        for (EMAComponentInstantiationSymbol inst : cmp.getSubComponents()) {
            Log.info(inst.getComponentType().getReferencedSymbol().howManyResolutionDeclarationSymbol() + "", "Important:");
            Log.debug(inst.toString(), "ComponentInstance CreateInstance PreSub");

            addSubComponents(inst, builder, filters, resolutionDeclarationSymbols, packageName);

            Log.debug(inst.getInstanceInformation().get().getInstanceNumberForArgumentIndex(0) + "", "InstanceInformation:");

            Log.debug(inst.toString(), "ComponentInstance CreateInstance PostSub");

        }

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
                                    .addResolvingFilters(filters)
                                    .addResolutionDeclarationSymbols(inst.getComponentType().getReferencedSymbol().getResolutionDeclarationSymbols())
                                    .addParameters(inst.getComponentType().getReferencedSymbol().getParameters())
                                    .build())

            );
            //Log.debug(superCmp.toString(), "superCmp post lambda");

        }

        return builder;
    }

    protected EMADynamicComponentInstanceBuilder getNewBuilder(){
        return EMADynamicComponentInstanceSymbol.builder();
    }

    protected void addSubComponents(EMAComponentInstantiationSymbol inst, EMADynamicComponentInstanceBuilder builder, Set<ResolvingFilter<? extends Symbol>> filters, List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols, String packageName){
        if(inst instanceof EMADynamicComponentInstantiationSymbol){
            EMADynamicComponentInstantiationSymbol dinst = (EMADynamicComponentInstantiationSymbol)inst;
            if(dinst.isDynamic()){
                for(int i = 0; i < dinst.getDimension(); ++i){
                    EMAComponentInstanceBuilder b = builderForComponentInstantiationSymbol(inst, filters, resolutionDeclarationSymbols, packageName);
                    b.setName(dinst.getNameWithoutArrayBracketPart()+"["+(i+1)+"]");

                    if(i >= dinst.getNonDynamicDimension() ){
                        if(b instanceof EMADynamicComponentInstanceBuilder){
                            b = ((EMADynamicComponentInstanceBuilder)b).addDynamicInstance(true);
                        }
                    }

                    builder.addSubComponent(b.build());
                }
                if(dinst.isDimensionInfinite()){
                    EMAComponentInstanceBuilder b = builderForComponentInstantiationSymbol(inst, filters, resolutionDeclarationSymbols, packageName);
                    b.setName(dinst.getNameWithoutArrayBracketPart()+"[oo]");
                    b = ((EMADynamicComponentInstanceBuilder)b).addDynamicInstance(true);
                    builder.addSubComponent(b.build());
                }
            }else if(dinst.isArray()) {
                for(int i = 0; i < dinst.getDimension(); ++i){
                    EMAComponentInstanceBuilder b = builderForComponentInstantiationSymbol(inst, filters, resolutionDeclarationSymbols, packageName);
                    b.setName(dinst.getNameWithoutArrayBracketPart()+"["+(i+1)+"]");
                    builder.addSubComponent(b.build());
                }
            }else{
//                Log.error("Something went wrong! Case not handled.");
                builder.addSubComponent(builderForComponentInstantiationSymbol(inst, filters, resolutionDeclarationSymbols, packageName).build());
            }
        }else{
            builder.addSubComponent(builderForComponentInstantiationSymbol(inst, filters, resolutionDeclarationSymbols, packageName).build());
        }
    }

    protected EMAComponentInstanceBuilder builderForComponentInstantiationSymbol(EMAComponentInstantiationSymbol inst, Set<ResolvingFilter<? extends Symbol>> filters, List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols, String packageName){
        return  createInstance(inst.getComponentType(), filters, inst.getComponentType().getReferencedSymbol().getResolutionDeclarationSymbols(), packageName + "." + inst.getName())
                .setName(inst.getName())
                .setPackageName(packageName)
                .addActualTypeArguments(inst.getComponentType().getFormalTypeParameters(), inst.getComponentType().getActualTypeArguments())
                .addResolvingFilters(filters)
                .addResolutionDeclarationSymbols(inst.getComponentType().getResolutionDeclarationSymbols())
                .addParameters(inst.getComponentType().getReferencedSymbol().getParameters())
                .addArguments(inst.getComponentType().getReferencedSymbol().getArguments());
    }
}
