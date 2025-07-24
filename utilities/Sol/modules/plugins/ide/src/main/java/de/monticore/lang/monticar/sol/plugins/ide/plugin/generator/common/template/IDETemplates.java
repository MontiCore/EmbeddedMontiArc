/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.template;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ModuleTypeSymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateContribution;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.TemplateRegistry;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.filter.LocalFilter;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable.IDESymbolTable;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable.LocalAwareIDESymbol;

import java.util.Set;

@Singleton
public class IDETemplates implements TemplateContribution {
    protected static final String FRONTEND = "Frontend";
    protected static final String BACKEND = "Backend";

    protected final IDESymbolTable symbolTable;
    protected final LocalFilter filter;

    @Inject
    protected IDETemplates(IDESymbolTable symbolTable, LocalFilter filter) {
        this.symbolTable = symbolTable;
        this.filter = filter;
    }

    @Override
    public void registerTemplates(TemplateRegistry registry) {
        LocalAwareIDESymbol ide = this.symbolTable.getRootSymbol()
                .map(core -> new LocalAwareIDESymbol(core, this.filter))
                .orElseThrow(() -> new RuntimeException("Root Symbol could not be located."));

        String ideName = ide.getName().toLowerCase();
        Set<ConfigurationTypeSymbol> configurations = this.filter.filter(ide.getConfigurationTypeInclusionSymbols());
        Set<ModuleTypeSymbol> modules = this.filter.filter(ide.getModuleTypeInclusionSymbols());

        registry.setTemplateRoot("templates/option");
        registry.setTopPatternSuffix("-top");

        configurations.forEach(configuration -> {
            String configurationName = configuration.getFullName().toLowerCase();

            registry.registerTemplate(
                    "common/protocol.ftl",
                    String.format("common/configurations/%s/protocol.ts", configurationName),
                    configuration
            );
        });

        registry.setTemplateRoot("templates/ide/common");

        registry.registerTemplate(
                "browser/ide-frontend-module.ftl",
                String.format("browser/%s-frontend-module.ts", ideName),
                ide
        );

        registry.registerTemplate(
                "browser/ide-module-type-contribution.ftl",
                String.format("browser/%s-module-type-contribution.ts", ideName),
                ide
        );

        registry.registerTemplate(
                "common/ide-end-runner-contribution.ftl",
                String.format("browser/%s-frontend-runner-contribution.ts", ideName),
                ide, FRONTEND
        );

        registry.registerTemplate(
                "common/ide-end-validator-contribution.ftl",
                String.format("browser/%s-frontend-validator-contribution.ts", ideName),
                ide, FRONTEND
        );

        registry.registerTemplate(
                "browser/ide-configuration-type-contribution.ftl",
                String.format("browser/%s-configuration-type-contribution.ts", ideName),
                ide
        );

        registry.registerTemplate(
                "node/ide-backend-module.ftl",
                String.format("node/%s-backend-module.ts", ideName),
                ide
        );

        registry.registerTemplate(
                "common/ide-end-runner-contribution.ftl",
                String.format("node/%s-backend-runner-contribution.ts", ideName),
                ide, BACKEND
        );

        registry.registerTemplate(
                "common/ide-end-validator-contribution.ftl",
                String.format("node/%s-backend-validator-contribution.ts", ideName),
                ide, BACKEND
        );

        registry.registerTemplate(
                "node/ide-coordinator-contribution.ftl",
                String.format("node/%s-coordinator-contribution.ts", ideName),
                ide
        );

        registry.registerTemplate(
                "node/ide-module-creator-contribution.ftl",
                String.format("node/%s-module-creator-contribution.ts", ideName),
                ide
        );

        registry.registerTemplate(
                "main/ide-docker-port-contribution.ftl",
                String.format("main/%s-docker-port-contribution.ts", ideName),
                ide
        );

        registry.registerTemplate(
                "main/ide-main-module.ftl",
                String.format("main/%s-main-module.ts", ideName),
                ide
        );

        configurations.forEach(configuration -> {
            String configurationFolder = configuration.getFullName().toLowerCase();

            registry.registerTemplate(
                    "common/configuration/configuration-end-validator.ftl",
                    String.format("browser/configurations/%s/frontend-validator.ts", configurationFolder),
                    configuration, FRONTEND
            );

            registry.registerTemplate(
                    "common/configuration/configuration-end-runner.ftl",
                    String.format("browser/configurations/%s/frontend-runner.ts", configurationFolder),
                    configuration, FRONTEND
            );

            registry.registerTemplate(
                    "common/configuration/configuration-end-validator.ftl",
                    String.format("node/configurations/%s/backend-validator.ts", configurationFolder),
                    configuration, BACKEND
            );

            registry.registerTemplate(
                    "common/configuration/configuration-end-runner.ftl",
                    String.format("node/configurations/%s/backend-runner.ts", configurationFolder),
                    configuration, BACKEND
            );

            registry.registerTemplate(
                    "node/configuration/configuration-coordinator.ftl",
                    String.format("node/configurations/%s/coordinator.ts", configurationFolder),
                    configuration
            );
        });

        modules.forEach(module -> {
            String moduleFolder = module.getFullName().toLowerCase();

            registry.registerTemplate(
                    "common/module/module-end-validator.ftl",
                    String.format("browser/modules/%s/frontend-validator.ts", moduleFolder),
                    module, FRONTEND
            );

            registry.registerTemplate(
                    "common/module/module-end-validator.ftl",
                    String.format("node/modules/%s/backend-validator.ts", moduleFolder),
                    module, BACKEND
            );

            registry.registerTemplate(
                    "node/module/module-creator.ftl",
                    String.format("node/modules/%s/creator.ts", moduleFolder),
                    module
            );
        });
    }
}
