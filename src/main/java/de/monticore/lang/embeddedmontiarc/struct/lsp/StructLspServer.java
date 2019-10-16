package de.monticore.lang.embeddedmontiarc.struct.lsp;
import de.monticore.util.lsp.McLspServer;
import de.monticore.util.lsp.MontiCoreDocumentService;

import java.io.IOException;
import java.util.concurrent.ExecutionException;

public class StructLspServer extends McLspServer {
    private StructDocumentService structDocumentService = new StructDocumentService();

    public static void main(String[] args) throws InterruptedException, ExecutionException, IOException {
        StructLspServer server = new StructLspServer();
        server.startFromArgs(args);
    }

    @Override
    public MontiCoreDocumentService getTextDocumentService() {
        return structDocumentService;
    }
}
