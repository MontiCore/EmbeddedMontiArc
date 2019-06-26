/*
 * Copyright (C) 2019 SE RWTH.
 *
 * TODO: Include License.
 */
package de.monticore.lang.monticar.sol.runtime.ls.services;

import de.monticore.lang.monticar.sol.runtime.ls.document.TextDocument;
import org.eclipse.lsp4j.Diagnostic;

import java.util.List;

public interface DiagnosticsService {
    /**
     * @param document The text document for which diagnostics should be computed.
     * @return Diagnostics of the given text document.
     */
    List<Diagnostic> validate(TextDocument document);
}
