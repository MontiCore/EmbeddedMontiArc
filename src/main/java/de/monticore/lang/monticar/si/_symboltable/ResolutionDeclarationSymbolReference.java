/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.si._symboltable;

import de.monticore.lang.monticar.resolution._ast.ASTResolution;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.references.SymbolReference;

/**
 * Created by Sascha on 14.05.2017.
 */
public class ResolutionDeclarationSymbolReference extends ResolutionDeclarationSymbol implements SymbolReference<ResolutionDeclarationSymbol> {
    protected int dimension = 0;

    public ResolutionDeclarationSymbolReference(final String name, String nameToResolve, ASTResolution astResolution) {
        super(name, nameToResolve, astResolution);
    }

    public static ResolutionDeclarationSymbolReference constructResolutionDeclSymbolRef(String nameToResolve,
                                                                                        ASTResolution astResolution) {

        return new ResolutionDeclarationSymbolReference("ResolutionDeclarationSymbol", nameToResolve, astResolution);
    }

    @Override
    public ResolutionDeclarationSymbol getReferencedSymbol() {
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
}
