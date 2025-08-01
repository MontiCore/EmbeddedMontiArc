<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.plugins.ls.plugin.configuration.LanguageServerConfiguration" -->
<#assign configuration = glex.getGlobalVar("configuration")>
<#assign generatedPackage = configuration.getGrammarGeneratedPackage()>
<#assign grammarName = configuration.getGrammarName()>
/*
 * (c) https://github.com/MontiCore/monticore
 */
package ${generatedPackage}.ls;

import com.google.inject.Guice;
import com.google.inject.Injector;
import ${generatedPackage}.${grammarName}Module;
import de.monticore.lang.monticar.sol.runtime.ls.ls.ServerLauncher;

public final class ${grammarName}ServerLauncher {
    public static void main(String[] arguments) throws Exception {
        Injector injector = Guice.createInjector(new ${grammarName}Module());
        ServerLauncher launcher = injector.getInstance(ServerLauncher.class);

        launcher.launch(arguments);
    }
}
