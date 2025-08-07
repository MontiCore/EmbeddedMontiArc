/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment._symboltable;

import com.google.inject.assistedinject.Assisted;
import com.google.inject.assistedinject.AssistedInject;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTDockerfile;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTEnvironmentCompilationUnit;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.Names;

import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;

public class EnvironmentSymbolTableCreator extends EnvironmentSymbolTableCreatorTOP {
    protected ASTEnvironmentCompilationUnit root;

    @AssistedInject
    public EnvironmentSymbolTableCreator(ResolvingConfiguration resolvingConfig, @Assisted MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    @AssistedInject
    public EnvironmentSymbolTableCreator(ResolvingConfiguration resolvingConfig, @Assisted Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }

    protected Optional<ASTEnvironmentCompilationUnit> getRootNode() {
        return Optional.ofNullable(this.root);
    }

    @Override
    protected void initialize_Dockerfile(DockerfileSymbol dockerfile, ASTDockerfile ast) {
        ast.setDockerfileSymbol(dockerfile);
        dockerfile.setComponent(ast.isComponent());
        this.getRootNode().ifPresent(node -> {
            node.forEachImports(i -> {
                String qualifiedName = Names.getQualifiedName(i.getImportList());
                DockerfileSymbolReference reference
                        = new DockerfileSymbolReference(qualifiedName, this.currentScope().orElse(null));

                dockerfile.addComponent(reference);
            });
        });
    }

    @Override
    public void visit(ASTEnvironmentCompilationUnit node) {
        List<ImportStatement> imports = new ArrayList<>();
        String packageName = Names.getQualifiedName(node.getPackageList());

        this.root = node;

        node.forEachImports(i -> {
            String qualifiedName = Names.getQualifiedName(i.getImportList());

            imports.add(new ImportStatement(qualifiedName, false));
        });

        this.putOnStack(new ArtifactScope(Optional.empty(), packageName, imports));
    }
}
