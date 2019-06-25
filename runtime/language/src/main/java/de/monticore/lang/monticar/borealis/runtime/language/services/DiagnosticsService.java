package de.monticore.lang.monticar.borealis.runtime.language.services;

import de.monticore.lang.monticar.borealis.runtime.language.document.TextDocument;
import org.eclipse.lsp4j.Diagnostic;

import java.util.List;

public interface DiagnosticsService {
    List<Diagnostic> validate(TextDocument document);
}
