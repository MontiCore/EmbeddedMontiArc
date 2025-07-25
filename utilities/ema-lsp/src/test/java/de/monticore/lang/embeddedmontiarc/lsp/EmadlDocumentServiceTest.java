package de.monticore.lang.embeddedmontiarc.lsp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcdl.lsp.EmadlDocumentService;
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

class EmadlDocumentServiceTest extends AbstractTextDocumentServiceTest {

    public static final String BASE_PATH = "src/test/resources/emadl";

    private EmadlDocumentService getDocumentService(String basePath) throws IOException {
        ModelFileCache modelFileCache = new ModelFileCache(Paths.get(basePath).toAbsolutePath(), Collections.singleton("emadl"));
        EmadlDocumentService res = new EmadlDocumentService(modelFileCache);
        res.setClient(getMockClient());
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
        EmadlDocumentService documentService = getDocumentService(BASE_PATH);
        DiagnosticsLog.setLogToStdout(true);
        documentService.doParse(new StringReader("package a; component Abc{}"));
        assertTrue(DiagnosticsLog.getFindings().isEmpty());
    }

    @Test
    public void testInvalidSyntax() throws InterruptedException, ExecutionException, IOException {
        EmadlDocumentService documentService = getDocumentService(BASE_PATH);
        documentService.doParse(new StringReader("package a; compoent Abc{}"));
        assertFalse(DiagnosticsLog.getFindings().isEmpty());
    }

    @Test
    public void testValidDidOpenEvent() throws IOException {
        EmadlDocumentService documentService = getDocumentService(BASE_PATH);
        DiagnosticsLog.init();
        DiagnosticsLog.setDebug(true);
        DiagnosticsLog.setTrace(true);
        File file = new File("src/test/resources/emadl/valid/SimpleComponent.emadl");
        documentService.didOpen(createDidOpenEvent(file, "EmbeddedMontiArcDL"));
        assertTrue(DiagnosticsLog.getFindings().isEmpty());
    }

    @Test
    public void testInvalidDidOpenEvent() throws IOException {
        EmadlDocumentService documentService = getDocumentService(BASE_PATH);
        File file = new File("src/test/resources/emadl/invalid/SimpleComponent.emadl");
        documentService.didOpen(createDidOpenEvent(file, "EmbeddedMontiArcDL"));
        assertFalse(DiagnosticsLog.getFindings().isEmpty());
    }

}