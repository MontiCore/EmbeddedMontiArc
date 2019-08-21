/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.validator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguage;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.symboltable.LanguageSymbolTable;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.plugin.Mojo;
import org.apache.maven.plugin.MojoExecutionException;

import java.util.Optional;

@Singleton
public class LDValidatorImpl implements LDValidator, PluginContribution {
    protected final NotificationService notifications;
    protected final LanguageSymbolTable symbolTable;
    protected final LanguageCoCoChecker checker;
    protected final NPMPackageService packages;

    @Inject
    protected LDValidatorImpl(NotificationService notifications, LanguageSymbolTable symbolTable,
                              LanguageCoCoChecker checker, NPMPackageService packages) {
        this.notifications = notifications;
        this.symbolTable = symbolTable;
        this.checker = checker;
        this.packages = packages;
    }

    @Override
    public void validate() throws Exception {
        this.validatePackage();
        this.validateModels();
    }

    @Override
    public int getPriority() {
        return 50;
    }

    @Override
    public void onPluginConfigure(Mojo plugin) throws Exception {
        this.validate();
    }

    protected void validatePackage() throws Exception {
        SolPackage solPackage = this.packages.getCurrentPackage()
                .orElseThrow(() -> new MojoExecutionException("Root Package could not be located or is not a Sol package."));

        if (!solPackage.isTheiaPackage()) throw new MojoExecutionException("Root Package should also be a Theia package.");
    }

    protected void validateModels() throws Exception {
        this.symbolTable.getRootNode().ifPresent(node -> this.validateNode(node.getLanguage()));

        if (Log.getFindings().size() > 0) throw new MojoExecutionException("There are erroneous models.");
    }

    protected void validateNode(ASTLanguage node) {
        this.checker.checkAll(node);
        node.getLanguageSymbolOpt().ifPresent(symbol -> {
            symbol.getParentSymbols().stream()
                    .map(LanguageSymbol::getLanguageNode)
                    .filter(Optional::isPresent)
                    .map(Optional::get)
                    .forEach(this::validateNode);
        });
    }
}
