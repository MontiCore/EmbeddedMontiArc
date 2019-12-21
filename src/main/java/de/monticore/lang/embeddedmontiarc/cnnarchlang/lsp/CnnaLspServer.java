package de.monticore.lang.embeddedmontiarc.cnnarchlang.lsp;
import de.monticore.util.lsp.McLspServer;
import de.monticore.util.lsp.MontiCoreDocumentService;

import java.io.IOException;
import java.util.concurrent.ExecutionException;

public class CnnaLspServer extends McLspServer {
    private CnnaDocumentService CnnaDocumentService = new CnnaDocumentService();

    public static void main(String[] args) throws InterruptedException, ExecutionException, IOException {
        CnnaLspServer server = new CnnaLspServer();
        server.startFromArgs(args);
    }

    @Override
    public MontiCoreDocumentService getTextDocumentService() {
        return CnnaDocumentService;
    }
}
