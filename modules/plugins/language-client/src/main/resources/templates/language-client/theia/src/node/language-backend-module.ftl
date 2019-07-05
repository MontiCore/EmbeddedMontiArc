<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration" -->
<#assign configuration = glex.getGlobalVar("configuration")>
<#assign grammarName = configuration.getGrammarName()>
<#assign grammarNameLC = grammarName?lower_case>
import { LanguageServerContribution } from "@theia/languages/lib/node";
import { ContainerModule } from "inversify";
import { ${grammarName}ServerContribution } from "./${grammarNameLC}-server-contribution";

export default new ContainerModule(bind => {
    bind(${grammarName}ServerContribution).toSelf().inSingletonScope();
    bind(LanguageServerContribution).toService(${grammarName}ServerContribution);
});
