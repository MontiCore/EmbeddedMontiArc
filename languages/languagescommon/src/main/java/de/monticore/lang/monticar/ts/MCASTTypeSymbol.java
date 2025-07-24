/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts;

import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Scope;
import de.monticore.types.types._ast.ASTType;

import java.util.Collections;
import java.util.List;
import java.util.Optional;

/**
 */
public class MCASTTypeSymbol extends CommonSymbol implements MCTypeSymbol {
    public static final MCASTTypeSymbolKind KIND;

    static {
        KIND = MCASTTypeSymbolKind.INSTANCE;
    }

    protected ASTType astType = null;

    public MCASTTypeSymbol(String name) {
        super(name, KIND);
    }

    public MCASTTypeSymbol(String name, ASTType astType) {
        super(name, KIND);
        this.astType = astType;
    }

    public MCASTTypeSymbol(String name, MutableScope enclosingScope, ASTType astType) {
        super(name, KIND);
        setEnclosingScope(enclosingScope);
        this.astType = astType;
    }

    public ASTType getAstType() {
        return astType;
    }

    public List<? extends MCTypeSymbol> getFormalTypeParameters() {
        return Collections.emptyList();
    }

    public Optional<? extends MCTypeReference<? extends MCTypeSymbol>> getSuperClass() {
        return Optional.empty();
    }

    public List<? extends MCTypeReference<? extends MCTypeSymbol>> getInterfaces() {
        return Collections.emptyList();
    }

    public List<? extends MCTypeReference<? extends MCTypeSymbol>> getSuperTypes() {
        return Collections.emptyList();
    }

    public boolean isFormalTypeParameter() {
        return false;
    }

    public Scope getSpannedScope() {
        return null;
    }
}
