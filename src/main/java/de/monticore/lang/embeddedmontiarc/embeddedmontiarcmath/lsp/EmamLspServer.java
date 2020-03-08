package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.lsp;
import de.monticore.util.lsp.McLspServer;
import de.monticore.util.lsp.ModelFileCache;
import de.monticore.util.lsp.MontiCoreDocumentService;

import java.io.IOException;
import java.util.Collections;
import java.util.concurrent.ExecutionException;

public class EmamLspServer extends McLspServer {
    private EmamDocumentService emamDocumentService = new EmamDocumentService(new ModelFileCache(Collections.singleton("emam")));

    public static void main(String[] args) throws InterruptedException, ExecutionException, IOException {
        EmamLspServer server = new EmamLspServer();
        server.startFromArgs(args);
    }

    @Override
    public MontiCoreDocumentService getTextDocumentService() {
        return emamDocumentService;
    }
}
