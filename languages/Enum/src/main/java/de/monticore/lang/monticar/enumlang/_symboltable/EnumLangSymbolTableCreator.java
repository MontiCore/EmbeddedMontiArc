/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.enumlang._symboltable;

import de.monticore.lang.monticar.enumlang._ast.ASTEnumLangCompilationUnit;
import de.monticore.lang.monticar.enumlang._cocos.EnumLangCoCoChecker;
import de.monticore.lang.monticar.enumlang.coco.DefaultEnumCoCoChecker;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.Names;

import java.util.Collections;
import java.util.Deque;

public class EnumLangSymbolTableCreator extends EnumLangSymbolTableCreatorTOP {

    private final EnumLangCoCoChecker coCoChecker = DefaultEnumCoCoChecker.create();

    public EnumLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    public EnumLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }

    @Override
    public void visit(ASTEnumLangCompilationUnit node) {
        coCoChecker.checkAll(node);
        String packageQualifiedName = Names.getQualifiedName(node.getPackageList());
        ArtifactScope artifactScope = new ArtifactScope(packageQualifiedName, Collections.emptyList());
        putOnStack(artifactScope);
    }
}
