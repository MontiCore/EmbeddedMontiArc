package de.monticore.lang.embeddedmontiarc.lsp;

import de.monticore.lang.embeddedmontiarc.cnntrainlang.lsp.CnntDocumentService;
import de.monticore.util.lsp.ModelFileCache;
import de.se_rwth.commons.logging.DiagnosticsLog;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.io.StringReader;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.concurrent.ExecutionException;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class CnntDocumentServiceTest extends AbstractTextDocumentServiceTest {

    public static final String BASE_PATH = "src/test/resources/cnnt";

    private CnntDocumentService getDocumentService(String basePath) throws IOException {
        CnntDocumentService res = new CnntDocumentService();
        res.setClient(getMockClient());
        res.setModelFileCache(new ModelFileCache(Paths.get(basePath).toAbsolutePath() , Collections.singleton(".cnnt")));
        return res;
    }

    @BeforeAll
    public static void setup(){
        DiagnosticsLog.init();
        DiagnosticsLog.getFindings().clear();
    }

    @AfterAll
    public static void reset(){
        DiagnosticsLog.getFindings().clear();
    }
    @Test
    public void testValidSyntax() throws InterruptedException, ExecutionException, IOException {
        CnntDocumentService documentService = getDocumentService(BASE_PATH);
        DiagnosticsLog.setLogToStdout(true);
        documentService.doParse(new StringReader("configuration SimpleConfigValid{}"));
        assertTrue(DiagnosticsLog.getFindings().isEmpty());
    }

    @Test
    public void testInvalidSyntax() throws InterruptedException, ExecutionException, IOException {
        CnntDocumentService documentService = getDocumentService(BASE_PATH);
        documentService.doParse(new StringReader("confiuration SimpleConfigInvalid{}"));
        assertFalse(DiagnosticsLog.getFindings().isEmpty());
    }

    @Test
    public void testValidDidOpenEvent() throws IOException {
        CnntDocumentService documentService = getDocumentService(BASE_PATH);

        File file = new File("src/test/resources/cnnt/valid/SimpleConfig.cnnt");
        documentService.didOpen(createDidOpenEvent(file, "CNNTrainLang"));
        assertTrue(DiagnosticsLog.getFindings().isEmpty());
    }

    @Test
    public void testInvalidDidOpenEvent() throws IOException {
        CnntDocumentService documentService = getDocumentService(BASE_PATH);
        File file = new File("src/test/resources/cnnt/invalid/SimpleConfig.cnnt");
        documentService.didOpen(createDidOpenEvent(file, "CNNTrainLang"));
        assertFalse(DiagnosticsLog.getFindings().isEmpty());
    }

}
