/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.generator.ddf.collector;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.environment._ast.*;
import de.monticore.lang.monticar.sol.grammars.environment.visitor.MultiEnvironmentVisitor;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.configuration.EnvironmentGenerateConfiguration;

import java.util.ArrayList;
import java.util.List;

@Singleton
public class DDFCollectorImpl implements DDFCollector, MultiEnvironmentVisitor {
    protected final NotificationService notifications;
    protected final EnvironmentGenerateConfiguration configuration;
    protected final List<ASTInstruction> instructions;

    @Inject
    protected DDFCollectorImpl(NotificationService notifications, EnvironmentGenerateConfiguration configuration) {
        this.notifications = notifications;
        this.configuration = configuration;
        this.instructions = new ArrayList<>();
    }

    @Override
    public List<ASTInstruction> collect() {
        this.instructions.clear();
        this.notifications.info("Collecting Instructions.");
        this.handle(this.configuration.getRootDirectory());
        return this.instructions;
    }

    @Override
    public void visit(ASTArgInstruction node) {
        this.instructions.add(node);
    }

    @Override
    public void visit(ASTEnvInstruction node) {
        this.instructions.add(node);
    }

    @Override
    public void visit(ASTExposeInstruction node) {
        this.instructions.add(node);
    }

    @Override
    public void visit(ASTInstallInstruction node) {
        this.instructions.add(node);
    }

    @Override
    public void visit(ASTLabelInstruction node) {
        this.instructions.add(node);
    }

    @Override
    public void visit(ASTRunInstruction node) {
        this.instructions.add(node);
    }

    @Override
    public void visit(ASTShellInstruction node) {
        this.instructions.add(node);
    }

    @Override
    public void visit(ASTVolumeInstruction node) {
        this.instructions.add(node);
    }

    @Override
    public void visit(ASTWorkDirInstruction node) {
        this.instructions.add(node);
    }
}
