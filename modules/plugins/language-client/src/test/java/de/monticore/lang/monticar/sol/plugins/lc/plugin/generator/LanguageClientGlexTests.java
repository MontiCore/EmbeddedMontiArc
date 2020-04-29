/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator;

import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.symboltable.LanguageSymbolTable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.util.Optional;

import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
public class LanguageClientGlexTests {
    @Mock LanguageClientConfiguration configuration;
    @Mock NPMPackageService packages;
    @Mock LanguageSymbolTable symbolTable;

    @InjectMocks LanguageClientGlex lcGlex;

    @Test
    void testDefineGlobalVars() {
        GlobalExtensionManagement glex = mock(GlobalExtensionManagement.class);
        SolPackage solPackage = mock(SolPackage.class);
        LanguageSymbol rootSymbol = mock(LanguageSymbol.class);

        when(packages.getCurrentPackage()).thenReturn(Optional.of(solPackage));
        when(symbolTable.getRootSymbol()).thenReturn(Optional.of(rootSymbol));

        lcGlex.defineGlobalVars(glex);

        verify(glex).defineGlobalVar("configuration", configuration);
        verify(glex).defineGlobalVar("rootPackage", solPackage);
        verify(glex).defineGlobalVar("rootSymbol", rootSymbol);
    }
}
