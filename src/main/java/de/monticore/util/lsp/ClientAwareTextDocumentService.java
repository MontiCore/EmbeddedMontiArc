/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp;

import org.eclipse.lsp4j.services.LanguageClient;
import org.eclipse.lsp4j.services.LanguageClientAware;
import org.eclipse.lsp4j.services.TextDocumentService;

public interface ClientAwareTextDocumentService extends TextDocumentService, LanguageClientAware {
    void setClient(LanguageClient client);
}
