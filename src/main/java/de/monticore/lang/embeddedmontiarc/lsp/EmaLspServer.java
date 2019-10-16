package de.monticore.lang.embeddedmontiarc.lsp;
import de.monticore.util.lsp.McLspServer;
import de.monticore.util.lsp.MontiCoreDocumentService;

import java.io.IOException;
import java.util.concurrent.ExecutionException;

public class EmaLspServer extends McLspServer {
    private EmaDocumentService emaDocumentService = new EmaDocumentService();

    public static void main(String[] args) throws InterruptedException, ExecutionException, IOException {
        EmaLspServer server = new EmaLspServer();
        server.startFromArgs(args);
    }

    @Override
    public MontiCoreDocumentService getTextDocumentService() {
        return emaDocumentService;
    }
}
