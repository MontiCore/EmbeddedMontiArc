/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl.adapter;

import de.monticore.lang.monticar.cnnarch._symboltable.ParameterSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.TransitiveAdaptedResolvingFilter;

public class ResolutionDeclarationSymbol2ParameterSymbolTypeFilter extends TransitiveAdaptedResolvingFilter<ParameterSymbol> {

    public ResolutionDeclarationSymbol2ParameterSymbolTypeFilter() {
        super(ResolutionDeclarationSymbol.KIND,
                ParameterSymbol.class,
                ParameterSymbol.KIND);
    }

    @Override
    public Symbol translate(Symbol adaptee) {
        assert adaptee instanceof ResolutionDeclarationSymbol;
        if (((ResolutionDeclarationSymbol) adaptee).getASTResolution() instanceof ASTUnitNumberResolution){
            return new ResolutionDeclarationSymbol2ParameterSymbol((ResolutionDeclarationSymbol) adaptee, ((ASTUnitNumberResolution) ((ResolutionDeclarationSymbol) adaptee).getASTResolution()));
        }
        return null;
    }
}
