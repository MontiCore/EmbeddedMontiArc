package de.monticore.lang.monticar.emadl.modularcnn.builder;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentScope;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.monticar.cnnarch._ast.CNNArchMill;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchSimpleExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ParameterSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ParameterType;
import de.monticore.lang.monticar.emadl._ast.EMADLMill;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.ts.MCFieldSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

public class InstanceBuilder extends EMAComponentInstanceBuilder {

    private ASTEMACompilationUnit node = null;

    public InstanceBuilder() {
        super();
    };
    public InstanceBuilder(ASTEMACompilationUnit node){
        super();
        this.node = node;
    }

    public void buildNode(ASTEMACompilationUnit node){
        this.node = node;
    }

    @Override
    public EMAComponentInstanceSymbol build(){

        EMAComponentInstanceSymbol instance = super.build();


        Optional<ArchitectureSymbol> arch = instance.getSpannedScope().resolve("",ArchitectureSymbol.KIND);

        Log.info("Instance after resolve: " + instance,"INSTANCE_BUILDER_BUILD");

        if (arch.isPresent()){
            addArchitectureParameterSymbolsToInstance(instance);
            Log.info("Instance after arch params: " + instance,"INSTANCE_BUILDER_BUILD");

        }

        Log.info("Instance Final: " + instance,"INSTANCE_BUILDER_BUILD");

        return instance;
    }

    public void addArchitectureParameterSymbolsToInstance(EMAComponentInstanceSymbol instance){
        for (ResolutionDeclarationSymbol symbol: instance.getResolutionDeclarationSymbols()){
            if (symbol.getASTResolution() instanceof ASTUnitNumberResolution){
                ASTUnitNumberResolution numberResolution = (ASTUnitNumberResolution) symbol.getASTResolution();
                ParameterSymbol parameterSymbol = new ParameterSymbol.Builder()
                        .name(symbol.getNameToResolve())
                        .type(ParameterType.ARCHITECTURE_PARAMETER)
                        .build();
                parameterSymbol.setExpression(ArchSimpleExpressionSymbol.of(numberResolution.getNumber().get()));
                instance.getSpannedScope().getAsMutableScope().add(parameterSymbol);
            } else {
                Log.error("Error while adding architecture generic parameters to composed CNN. Instance name: " + instance.getName());
            }
        }

        for (int i = 0; i < instance.getArguments().size(); i++){
            if (instance.getArguments().get(i) instanceof ASTNumberExpression){
                ASTNumberExpression expression = (ASTNumberExpression) instance.getArguments().get(i);

                MCFieldSymbol emaParam = instance.getComponentType().getConfigParameters().get(i);
                ParameterSymbol archParam = new ParameterSymbol.Builder()
                        .name(emaParam.getName())
                        .type(ParameterType.ARCHITECTURE_PARAMETER)
                        .build();
                archParam.setExpression(ArchSimpleExpressionSymbol.of(expression.getNumberWithUnit().getNumber().get()));

                instance.getSpannedScope().getAsMutableScope().add(archParam);
            } else {
                Log.error("Error while adding architecture configuration parameters to composed CNN. Instance name: " + instance.getName());
            }
        }
    }

    @Override
    protected void addOtherToComponentInstance(EMAComponentInstanceSymbol symbol){
        EMAComponentSymbol componentSymbol = symbol.getComponentType().getReferencedSymbol();
        Optional<ArchitectureSymbol> architectureSymbol = componentSymbol.getSpannedScope().resolve("",ArchitectureSymbol.KIND);

        if (architectureSymbol.isPresent()){
            ArchitectureSymbol newArchitectureSymbol = architectureSymbol.get().preResolveDeepCopy(symbol.getSpannedScope());
            symbol.getSpannedScope().getAsMutableScope().add(newArchitectureSymbol);
        }
    }
}
