/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts.references;

import de.monticore.lang.monticar.ts.MCASTTypeSymbol;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.types.references.ActualTypeArgument;
import de.monticore.types.types._ast.ASTType;

import java.util.Collections;
import java.util.List;

/**
 */
public class MCASTTypeSymbolReference extends MCASTTypeSymbol implements MCTypeReference<MCASTTypeSymbol> {
    public MCASTTypeSymbolReference(String name) {
        super(name);
    }

    public MCASTTypeSymbolReference(String name, ASTType astType) {
        super(name, astType);
    }

    public MCASTTypeSymbolReference(String name, MutableScope enclosingScope, ASTType astType) {
        super(name, enclosingScope, astType);
    }

    public static MCASTTypeSymbolReference constructReference(String name, MutableScope enclosingScope, ASTType astType) {
        MCASTTypeSymbolReference astTypeSymbolRef = new MCASTTypeSymbolReference(name, enclosingScope, astType);
        return astTypeSymbolRef;
    }

    public MCASTTypeSymbol getReferencedSymbol() {
        return this;
    }

    @Override
    public boolean existsReferencedSymbol() {
        return true;
    }

    @Override
    public boolean isReferencedSymbolLoaded() {
        return true;
    }

    @Override
    public AccessModifier getAccessModifier() {
        return this.getReferencedSymbol().getAccessModifier();
    }

    @Override
    public void setAccessModifier(AccessModifier accessModifier) {
        this.getReferencedSymbol().setAccessModifier(accessModifier);
    }

    @Override
    public int getDimension() {
        return 0;
    }

    @Override
    public void setDimension(int dimension) {

    }

    @Override
    public List<ActualTypeArgument> getActualTypeArguments() {
        return Collections.emptyList();
    }

    @Override
    public void setActualTypeArguments(List<ActualTypeArgument> list) {
    }
}
