package de.monticore.lang.monticar.sol.runtime.language.services;

import de.monticore.lang.monticar.sol.runtime.language.document.TextDocument;
import org.eclipse.lsp4j.Diagnostic;

import java.util.List;

public interface DiagnosticsService {
    /**
     * @param document The text document for which diagnostics should be computed.
     * @return Diagnostics of the given text document.
     */
    List<Diagnostic> validate(TextDocument document);
}
