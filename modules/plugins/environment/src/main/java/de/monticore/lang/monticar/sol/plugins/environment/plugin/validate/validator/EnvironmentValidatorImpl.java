/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.validator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTEnvironmentCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.environment._ast.EnvironmentMill;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.environment.visitor.MultiEnvironmentVisitor;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.PluginContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.configuration.EnvironmentValidateConfiguration;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.plugin.Mojo;

import java.io.File;

@Singleton
public class EnvironmentValidatorImpl implements EnvironmentValidator, PluginContribution, MultiEnvironmentVisitor {
    protected final NotificationService notifications;
    protected final EnvironmentValidateConfiguration configuration;
    protected final EnvironmentCoCoChecker checker;
    protected final ASTEnvironmentCompilationUnit ast;

    protected boolean hasFailed;

    @Inject
    protected EnvironmentValidatorImpl(NotificationService notifications, EnvironmentValidateConfiguration configuration,
                                       EnvironmentCoCoChecker checker) {
        this.notifications = notifications;
        this.configuration = configuration;
        this.checker = checker;
        this.ast = EnvironmentMill.environmentCompilationUnitBuilder().build();
        this.hasFailed = false;
    }

    protected void fail() {
        this.hasFailed = true;

        Log.getFindings().clear();
    }

    @Override
    public void validate() {
        Log.enableFailQuick(false);
        this.notifications.info("Validating Models.");
        this.handle(this.configuration.getRootDirectory());
    }

    @Override
    public int getPriority() {
        return 50;
    }

    @Override
    public void onPluginExecute(Mojo plugin) {
        this.validate();
    }

    @Override
    public void endVisit(File rootDirectory) {
        this.notifications.info("Checking overall Semantics.");
        this.checker.checkAll(this.ast);

        if (Log.getFindings().size() > 0) this.fail();

        if (this.hasFailed) throw new RuntimeException("Erroneous Models");
    }

    @Override
    public void visitModel(File model) {
        this.notifications.info("Checking Model '%s' for Syntax Errors.", model);
    }

    @Override
    public void endVisitModel(File model) {
        if (Log.getFindings().size() > 0) this.fail();
    }

    @Override
    public void visit(ASTEnvironmentCompilationUnit node) {
        this.ast.addAllInstructions(node.getInstructionList());
    }
}
