/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */

package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.Scope;

public abstract class VariableDeclarationSymbol extends CommonSymbol {

    public static final VariableDeclarationKind KIND = new VariableDeclarationKind();

    protected VariableDeclarationSymbol(String name) {
        super(name, KIND);
    }

    public abstract void putInScope(Scope scope);

    public abstract VariableDeclarationSymbol preResolveDeepCopy();

}
