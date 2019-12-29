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
import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

public class CheckArgument implements CNNArchASTArchArgumentCoCo {

    @Override
    public void check(ASTArchArgument node) {
        ArgumentSymbol argument = (ArgumentSymbol) node.getSymbolOpt().get();

        if(argument.getEnclosingScope().getSpanningSymbol().get() instanceof LayerSymbol) {
            LayerDeclarationSymbol layerDeclaration = argument.getLayer().getDeclaration();
            if (layerDeclaration != null && argument.getParameter() ==  null){
                Log.error("0"+ ErrorCodes.UNKNOWN_ARGUMENT + " Unknown Argument. " +
                                "Parameter with name '" + node.getName() + "' does not exist. " +
                                "Possible arguments are: " + Joiners.COMMA.join(layerDeclaration.getParameters())
                        , node.get_SourcePositionStart());
            }
        }else if(argument.getEnclosingScope().getSpanningSymbol().get() instanceof UnrollInstructionSymbol){
            UnrollDeclarationSymbol layerDeclaration = argument.getUnroll().getDeclaration();
            if (layerDeclaration != null && argument.getParameter() ==  null){
                Log.error("0"+ ErrorCodes.UNKNOWN_ARGUMENT + " Unknown Argument. " +
                                "Parameter with name '" + node.getName() + "' does not exist. " +
                                "Possible arguments are: " + Joiners.COMMA.join(layerDeclaration.getParameters())
                        , node.get_SourcePositionStart());
            }
        }
    }

}
