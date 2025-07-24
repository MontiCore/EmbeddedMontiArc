/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.collector;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.environment._ast.*;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.DockerfileSymbol;
import de.monticore.lang.monticar.sol.grammars.environment._visitor.EnvironmentInheritanceVisitor;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.symboltable.EnvironmentSymbolTable;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

@Singleton
public class ECCollectorImpl implements ECCollector, EnvironmentInheritanceVisitor {
    protected final NotificationService notifications;
    protected final EnvironmentSymbolTable symbolTable;
    protected final List<ASTInstruction> instructions;

    @Inject
    protected ECCollectorImpl(NotificationService notifications, EnvironmentSymbolTable symbolTable) {
        this.notifications = notifications;
        this.symbolTable = symbolTable;
        this.instructions = new ArrayList<>();
    }

    @Override
    public List<ASTInstruction> collect() {
        this.instructions.clear();
        this.notifications.info("Collecting Instructions.");
        this.symbolTable.getRootNode().ifPresent(this::handle);
        return this.instructions;
    }

    @Override
    public void visit(ASTDockerfile node) {
        node.getDockerfileSymbol().getComponentSymbols().stream()
                .map(DockerfileSymbol::getDockerfileNode)
                .map(Optional::get)
                .forEach(this::handle);
    }

    @Override
    public void visit(ASTInstruction node) {
        if (node instanceof ASTFrom) this.instructions.add(0, node);
        else this.instructions.add(node);
    }

    @Override
    public void traverse(ASTOnBuild node) { /* Deactivate traversal. */ }
}
