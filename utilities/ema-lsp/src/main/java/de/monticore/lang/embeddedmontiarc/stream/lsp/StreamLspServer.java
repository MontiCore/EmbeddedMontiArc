package de.monticore.lang.embeddedmontiarc.stream.lsp;
import de.monticore.util.lsp.McLspServer;
import de.monticore.util.lsp.MontiCoreDocumentService;

import java.io.IOException;
import java.util.concurrent.ExecutionException;

public class StreamLspServer extends McLspServer {
    private StreamDocumentService streamDocumentService = new StreamDocumentService();

    public static void main(String[] args) throws InterruptedException, ExecutionException, IOException {
        StreamLspServer server = new StreamLspServer();
        server.startFromArgs(args);
    }

    @Override
    public MontiCoreDocumentService getTextDocumentService() {
        return streamDocumentService;
    }
}
