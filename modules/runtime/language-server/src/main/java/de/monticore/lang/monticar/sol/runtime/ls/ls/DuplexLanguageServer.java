/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.ls;

import org.eclipse.lsp4j.jsonrpc.validation.NonNull;
import org.eclipse.lsp4j.services.LanguageClient;
import org.eclipse.lsp4j.services.LanguageServer;

public interface DuplexLanguageServer extends LanguageServer {
    /**
     * Associates a client with the server.
     * @param client The client which should be associated with the server.
     */
    void setRemoteProxy(@NonNull LanguageClient client);

    /**
     * @return The client associated with the server.
     */
    LanguageClient getRemoteProxy();
}
