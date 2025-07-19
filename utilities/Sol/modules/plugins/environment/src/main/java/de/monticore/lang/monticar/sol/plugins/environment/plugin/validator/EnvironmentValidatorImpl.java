/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.validator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTDockerfile;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.DockerfileSymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.symboltable.EnvironmentSymbolTable;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.plugin.Mojo;

import java.util.Optional;

@Singleton
public class EnvironmentValidatorImpl implements EnvironmentValidator, PluginContribution {
    protected final NotificationService notifications;
    protected final EnvironmentSymbolTable symbolTable;
    protected final EnvironmentCoCoChecker checker;

    @Inject
    protected EnvironmentValidatorImpl(NotificationService notifications, EnvironmentSymbolTable symbolTable,
                                       EnvironmentCoCoChecker checker) {
        this.notifications = notifications;
        this.symbolTable = symbolTable;
        this.checker = checker;
    }

    @Override
    public void validate() {
        Log.enableFailQuick(false);
        this.notifications.info("Validating Models.");
        this.symbolTable.getRootNode().ifPresent(node -> this.validate(node.getMain()));

        if (Log.getFindings().size() > 0) throw new RuntimeException("Erroneous Models");
    }

    @Override
    public int getPriority() {
        return 60;
    }

    @Override
    public void onPluginExecute(Mojo plugin) {
        this.validate();
    }

    protected void validate(ASTDockerfile node) {
        this.checker.checkAll(node);
        node.getDockerfileSymbolOpt().ifPresent(symbol -> {
            symbol.getComponentSymbols().stream()
                    .map(DockerfileSymbol::getDockerfileNode)
                    .filter(Optional::isPresent)
                    .map(Optional::get)
                    .forEach(this::validate);
        });
    }
}
