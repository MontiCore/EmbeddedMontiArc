<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.plugins.ls.plugin.configuration.LanguageServerConfiguration" -->
${tc.signature("template")}
<#assign configuration = glex.getGlobalVar("configuration")>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign generatedPackage = configuration.getGrammarGeneratedPackage()>
<#assign grammarName = configuration.getGrammarName()>
<#assign generatedPackage = configuration.getGrammarGeneratedPackage()>
/*
 * (c) https://github.com/MontiCore/monticore
 */
package ${generatedPackage}.services;

import com.google.common.flogger.FluentLogger;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.runtime.ls.converters.mc2ls.FindingToDiagnostic;
import de.monticore.lang.monticar.sol.runtime.ls.document.TextDocument;
import de.monticore.lang.monticar.sol.runtime.ls.services.DiagnosticsService;
import ${generatedPackage}._parser.${grammarName}Parser;
import de.se_rwth.commons.logging.Log;
import org.eclipse.lsp4j.Diagnostic;

import java.io.IOException;
import java.util.List;
import java.util.stream.Collectors;

<#if !hasHandwrittenPeer>@Singleton</#if>
public class ${grammarName}DiagnosticsService<#if hasHandwrittenPeer>TOP</#if> implements DiagnosticsService {
    protected final FluentLogger logger;
    protected final ${grammarName}Parser parser;
    protected final FindingToDiagnostic f2d;

    <#if !hasHandwrittenPeer>@Inject</#if>
    protected ${grammarName}DiagnosticsService<#if hasHandwrittenPeer>TOP</#if>(FindingToDiagnostic f2d) {
        this.logger = FluentLogger.forEnclosingClass();
        this.parser = new ${grammarName}Parser();
        this.f2d = f2d;
    }

    @Override
    public List<Diagnostic> validate(TextDocument document) {
        Log.getFindings().clear();

        try { this.parser.parse_String(document.getText()); } catch(IOException ignored) {}

        return Log.getFindings().stream().map(this.f2d).collect(Collectors.toList());
    }
}
