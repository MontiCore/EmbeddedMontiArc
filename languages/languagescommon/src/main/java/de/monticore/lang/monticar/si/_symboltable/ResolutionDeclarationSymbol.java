/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.si._symboltable;

import de.monticore.lang.monticar.resolution._ast.ASTResolution;
import de.monticore.symboltable.CommonSymbol;

/**
 * Created by Sascha on 14.05.2017.
 */
public class ResolutionDeclarationSymbol extends CommonSymbol {
    public static final ResolutionDeclarationSymbolKind KIND = ResolutionDeclarationSymbolKind.INSTANCE;
    protected ASTResolution astResolution;
    protected String nameToResolve;

    public ResolutionDeclarationSymbol(String name, String nameToResolve, ASTResolution astResolution) {
        super(name, KIND);
        this.nameToResolve = nameToResolve;
        this.astResolution = astResolution;
    }

    public ASTResolution getASTResolution() {
        return this.astResolution;
    }

    public String getNameToResolve() {
        return this.nameToResolve;
    }
}
