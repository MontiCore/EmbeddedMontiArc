/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

/**
 */
public class MathVariableDeclarationResolvingFilter extends CommonResolvingFilter<MathVariableDeclarationSymbol> {
    public MathVariableDeclarationResolvingFilter() {
        super(MathVariableDeclarationSymbol.KIND);
    }
}
