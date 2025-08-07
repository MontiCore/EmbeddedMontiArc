/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.artifact.plugin.validator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.artifact._cocos.ArtifactCoCoChecker;
import de.monticore.lang.monticar.sol.plugins.artifact.plugin.symboltable.ArtifactSymbolTable;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.plugin.Mojo;
import org.apache.maven.plugin.MojoExecutionException;

import java.util.List;
import java.util.stream.Collectors;

@Singleton
public class ArtifactValidatorImpl implements ArtifactValidator, PluginContribution {
    protected final NotificationService notifications;
    protected final ArtifactSymbolTable symbolTable;
    protected final ArtifactCoCoChecker checker;
    protected final NPMPackageService packages;

    @Inject
    protected ArtifactValidatorImpl(NotificationService notifications, ArtifactSymbolTable symbolTable,
                                    ArtifactCoCoChecker checker, NPMPackageService packages) {
        this.notifications = notifications;
        this.symbolTable = symbolTable;
        this.checker = checker;
        this.packages = packages;
    }

    @Override
    public void validate() throws MojoExecutionException {
        this.validatePackage();
        this.validateModels();
    }

    protected void validatePackage() throws MojoExecutionException {
        this.notifications.info("Checking package.");

        SolPackage solPackage = this.packages.getCurrentPackage()
                .orElseThrow(() -> new MojoExecutionException("Root Package could not be located or is not a Sol package."));

        if (!solPackage.isTheiaPackage()) throw new MojoExecutionException("Root Package should also be a Theia package.");
    }

    protected void validateModels() throws MojoExecutionException {
        this.notifications.info("Checking models.");
        this.symbolTable.getRootNodes().forEach(this.checker::checkAll);

        List<Finding> errors = Log.getFindings().stream()
                .filter(Finding::isError)
                .collect(Collectors.toList());

        if (!errors.isEmpty()) throw new MojoExecutionException("There are erroneous models.");
    }

    @Override
    public int getPriority() {
        return 65;
    }

    @Override
    public void onPluginConfigure(Mojo plugin) throws MojoExecutionException {
        this.notifications.info("Validating Environment.");
        this.validate();
    }
}
