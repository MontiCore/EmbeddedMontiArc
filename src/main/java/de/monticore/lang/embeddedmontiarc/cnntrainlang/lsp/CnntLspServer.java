package de.monticore.lang.embeddedmontiarc.cnntrainlang.lsp;
import de.monticore.util.lsp.McLspServer;
import de.monticore.util.lsp.ModelFileCache;
import de.monticore.util.lsp.MontiCoreDocumentService;

import java.io.IOException;
import java.util.Collections;
import java.util.concurrent.ExecutionException;

public class CnntLspServer extends McLspServer {
    private CnntDocumentService CnntDocumentService = new CnntDocumentService(new ModelFileCache(Collections.singleton("cnnt")));

    public static void main(String[] args) throws InterruptedException, ExecutionException, IOException {
        CnntLspServer server = new CnntLspServer();
        server.startFromArgs(args);
    }

    @Override
    public MontiCoreDocumentService getTextDocumentService() {
        return CnntDocumentService;
    }
}
