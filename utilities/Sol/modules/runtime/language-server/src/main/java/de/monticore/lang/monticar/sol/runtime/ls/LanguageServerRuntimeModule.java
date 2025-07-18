/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.runtime.ls.converters.ls2mc.DiagnosticToFinding;
import de.monticore.lang.monticar.sol.runtime.ls.converters.ls2mc.RangeToSourcePositions;
import de.monticore.lang.monticar.sol.runtime.ls.converters.mc2ls.FindingToDiagnostic;
import de.monticore.lang.monticar.sol.runtime.ls.converters.mc2ls.SourcePositionsToRange;
import de.monticore.lang.monticar.sol.runtime.ls.document.DefaultTextDocumentService;
import de.monticore.lang.monticar.sol.runtime.ls.ls.*;
import de.monticore.lang.monticar.sol.runtime.ls.options.*;
import de.monticore.lang.monticar.sol.runtime.ls.workspace.DefaultWorkspaceService;
import org.eclipse.lsp4j.services.TextDocumentService;
import org.eclipse.lsp4j.services.WorkspaceService;

public class LanguageServerRuntimeModule extends AbstractModule {
    @Override
    public void configure() {
        Multibinder<ServerLauncherContribution> additions =
                Multibinder.newSetBinder(binder(), ServerLauncherContribution.class);
        Multibinder<OptionsContribution> options =
                Multibinder.newSetBinder(binder(), OptionsContribution.class);

        bind(DiagnosticToFinding.class);
        bind(RangeToSourcePositions.class);
        bind(SourcePositionsToRange.class);
        bind(FindingToDiagnostic.class);

        bind(ServerLauncher.class).to(DefaultServerLauncher.class);

        bind(OptionsRegistry.class).to(DefaultOptionsRegistry.class);
        bind(OptionsService.class).to(DefaultOptionsService.class);

        bind(DuplexLanguageServer.class).to(DefaultDuplexLanguageServer.class);

        bind(WorkspaceService.class).to(DefaultWorkspaceService.class);

        bind(TextDocumentService.class).to(DefaultTextDocumentService.class);

        additions.addBinding().to(DefaultOptionsRegistry.class);
        additions.addBinding().to(DefaultOptionsService.class);

        options.addBinding().to(PortOption.class);
    }
}
