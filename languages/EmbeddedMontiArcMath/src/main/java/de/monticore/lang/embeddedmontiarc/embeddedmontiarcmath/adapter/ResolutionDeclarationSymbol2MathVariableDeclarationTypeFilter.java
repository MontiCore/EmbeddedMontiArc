/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.adapter;

import de.monticore.lang.math._symboltable.MathVariableDeclarationSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.TransitiveAdaptedResolvingFilter;
import de.se_rwth.commons.logging.Log;

/**
 */
public class ResolutionDeclarationSymbol2MathVariableDeclarationTypeFilter extends TransitiveAdaptedResolvingFilter<MathVariableDeclarationSymbol> {


    public ResolutionDeclarationSymbol2MathVariableDeclarationTypeFilter() {
        super(ResolutionDeclarationSymbol.KIND,
                MathVariableDeclarationSymbol.class,
                MathVariableDeclarationSymbol.KIND);
    }

    @Override
    public Symbol translate(Symbol adaptee) {
        if(adaptee instanceof  ResolutionDeclarationSymbol) {
            assert adaptee instanceof ResolutionDeclarationSymbol;
            if (((ResolutionDeclarationSymbol) adaptee).getASTResolution() instanceof ASTUnitNumberResolution)
                return new ResolutionDeclarationSymbol2MathVariableDeclarationSymbol((ResolutionDeclarationSymbol) adaptee, ((ASTUnitNumberResolution) ((ResolutionDeclarationSymbol) adaptee).getASTResolution()));
        }else{

            Log.debug(adaptee.toString(),"ResolutionDeclarationSymbol Adaption");
        }
        return null;
    }
}
