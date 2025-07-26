package de.monticore.lang.embeddedmontiarc.embeddedmontiarcdl.lsp;
import de.monticore.util.lsp.McLspServer;
import de.monticore.util.lsp.ModelFileCache;
import de.monticore.util.lsp.MontiCoreDocumentService;

import java.io.IOException;
import java.util.Collections;
import java.util.concurrent.ExecutionException;

public class EmadlLspServer extends McLspServer {
    private EmadlDocumentService emadlDocumentService = new EmadlDocumentService(new ModelFileCache(Collections.singleton("emadl")));

    public static void main(String[] args) throws InterruptedException, ExecutionException, IOException {
        EmadlLspServer server = new EmadlLspServer();
        server.startFromArgs(args);
    }

    @Override
    public MontiCoreDocumentService getTextDocumentService() {
        return emadlDocumentService;
    }
}
