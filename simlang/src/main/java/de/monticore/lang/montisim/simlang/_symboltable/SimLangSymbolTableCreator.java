package simlang._symboltable;

import static java.util.Objects.requireNonNull;

import java.util.ArrayList;
import java.util.Optional;

import simlang._ast.ASTSimLangCompilationUnit;

import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.ImportStatement;

import de.se_rwth.commons.Names;

import java.util.List;
import java.util.stream.Collectors;
import java.util.Deque;

public class SimLangSymbolTableCreator extends SimLangSymbolTableCreatorTOP {
  
  public SimLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
    super(resolvingConfig, enclosingScope);
  }
  public SimLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
  }
  
  @Override
  public void visit(ASTSimLangCompilationUnit node) {
    //coCoChecker.checkAll(node);
    String packageQualifiedName = Names.getQualifiedName(node.getPackage());
    List<ImportStatement> imports = node.getImportStatements()
            .stream()
            .map(imprt -> {
                String qualifiedImport = Names.getQualifiedName(imprt.getImportList());
                return new ImportStatement(qualifiedImport, imprt.isStar());
            })
            .collect(Collectors.toList());
    ArtifactScope artifactScope = new ArtifactScope(packageQualifiedName, imports);
    putOnStack(artifactScope);
  }
}