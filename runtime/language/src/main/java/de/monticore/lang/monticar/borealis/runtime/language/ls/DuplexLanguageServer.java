package de.monticore.lang.monticar.borealis.runtime.language.ls;

import org.eclipse.lsp4j.services.LanguageClient;
import org.eclipse.lsp4j.services.LanguageServer;

public interface DuplexLanguageServer extends LanguageServer {
    void setRemoteProxy(LanguageClient client);
    LanguageClient getRemoteProxy();
}
