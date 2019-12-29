/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._ast.ASTArchArgument;
import de.monticore.lang.monticar.cnnarch._ast.ASTLayer;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerDeclarationSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ParameterSymbol;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.Set;

public class CheckLayer implements CNNArchASTLayerCoCo{

    @Override
    public void check(ASTLayer node) {
        Set<String> nameSet = new HashSet<>();
        for (ASTArchArgument argument : node.getArgumentsList()){
            String name = argument.getName();
            if (nameSet.contains(name)){
                Log.error("0" + ErrorCodes.DUPLICATED_ARG + " Duplicated name: " + name +
                                ". Multiple values assigned to the same argument."
                        , argument.get_SourcePositionStart());
            }
            else {
                nameSet.add(name);
            }
        }
        LayerDeclarationSymbol layerDeclaration = ((LayerSymbol) node.getSymbolOpt().get()).getDeclaration();
        if (layerDeclaration == null){
            ArchitectureSymbol architecture = node.getSymbolOpt().get().getEnclosingScope().<ArchitectureSymbol>resolve("", ArchitectureSymbol.KIND).get();
            Log.error("0" + ErrorCodes.UNKNOWN_LAYER + " Unknown layer. " +
                            "Layer with name '" + node.getName() + "' does not exist. " +
                            "Existing layers: " + Joiners.COMMA.join(architecture.getLayerDeclarations()) + "."
                    , node.get_SourcePositionStart());
        }
        else {
            Set<String> requiredArguments = new HashSet<>();
            for (ParameterSymbol param : layerDeclaration.getParameters()){
                if (!param.getDefaultExpression().isPresent()){
                    requiredArguments.add(param.getName());
                }
            }
            for (ASTArchArgument argument : node.getArgumentsList()){
                requiredArguments.remove(argument.getName());
            }

            for (String missingArgumentName : requiredArguments){
                Log.error("0"+ErrorCodes.MISSING_ARGUMENT + " Missing argument. " +
                                "The argument '" + missingArgumentName + "' is required."
                        , node.get_SourcePositionStart());
            }
        }
    }

}
