<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.plugins.ls.plugin.configuration.LanguageServerConfiguration" -->
${tc.signature("template")}
<#assign configuration = glex.getGlobalVar("configuration")>
<#assign generatedPackage = configuration.getGrammarGeneratedPackage()>
<#assign grammarName = configuration.getGrammarName()>
/*
 * (c) https://github.com/MontiCore/monticore
 */
package ${generatedPackage};

import com.google.inject.AbstractModule;
import ${generatedPackage}.services.${grammarName}DiagnosticsService;
import de.monticore.lang.monticar.sol.runtime.ls.LanguageServerRuntimeModule;
import de.monticore.lang.monticar.sol.runtime.ls.services.DiagnosticsService;

public class ${grammarName}Module<#if template.hasHandwrittenPeer()>TOP</#if> extends AbstractModule {
    @Override
    public void configure() {
        bind(DiagnosticsService.class).to(${grammarName}DiagnosticsService.class);

        this.install(new LanguageServerRuntimeModule());
    }
}
