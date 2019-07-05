<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration" -->
<#assign configuration = glex.getGlobalVar("configuration")>
<#assign grammarName = configuration.getGrammarName()>
<#assign grammarNameLC = grammarName?lower_case>
import { LanguageClientContribution } from "@theia/languages/lib/browser";
import { ContainerModule } from "inversify";
import { ${grammarName}ClientContribution } from "./${grammarNameLC}-client-contribution";
import { ${grammarName}GrammarContribution } from "./${grammarNameLC}-grammar-contribution";
import { LanguageGrammarDefinitionContribution } from "@theia/monaco/lib/browser/textmate";

export default new ContainerModule(bind => {
    bind(${grammarName}GrammarContribution).toSelf().inSingletonScope();
    bind(LanguageGrammarDefinitionContribution).to(${grammarName}GrammarContribution).inSingletonScope();

    bind(${grammarName}ClientContribution).toSelf().inSingletonScope();
    bind(LanguageClientContribution).toService(${grammarName}ClientContribution);
});
