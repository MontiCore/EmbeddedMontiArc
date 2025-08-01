package de.monticore.lang.embeddedmontiarc.lsp;

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

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class EmaDocumentServiceTest extends AbstractTextDocumentServiceTest {

    public static final String BASE_PATH = "src/test/resources/ema";

    private EmaDocumentService getDocumentService(String basePath){
        ModelFileCache modelFileCache = new ModelFileCache(Paths.get(basePath).toAbsolutePath(), Collections.singleton("ema"));
        EmaDocumentService res = new EmaDocumentService(modelFileCache);
        res.setClient(getMockClient());
        return res;
    }

    @BeforeAll
    public static void setup(){
        DiagnosticsLog.init();
    }

    @AfterAll
    public static void reset(){
        DiagnosticsLog.getFindings().clear();
    }

    @Test
    public void testValidSyntax() throws IOException {
        EmaDocumentService documentService = getDocumentService(BASE_PATH);
        documentService.doParse(new StringReader("package a; component Abc{}"));
        assertTrue(DiagnosticsLog.getFindings().isEmpty());
    }

    @Test
    public void testInvalidSyntax() throws IOException {
        EmaDocumentService documentService = getDocumentService(BASE_PATH);
        documentService.doParse(new StringReader("package a; compoent Abc{}"));
        assertFalse(DiagnosticsLog.getFindings().isEmpty());
    }

    @Test
    public void testValidDidOpenEvent() throws IOException {
        EmaDocumentService documentService = getDocumentService(BASE_PATH);
        DiagnosticsLog.setLogToStdout(true);
        File file = new File("src/test/resources/ema/valid/SimpleComponent.ema");
        documentService.didOpen(createDidOpenEvent(file, "EmbeddedMontiArc"));
        assertTrue(DiagnosticsLog.getFindings().isEmpty());
    }

    @Test
    public void testInvalidDidOpenEvent() throws IOException {
        EmaDocumentService documentService = getDocumentService(BASE_PATH);

        File file = new File("src/test/resources/ema/invalid/SimpleComponent.ema");
        documentService.didOpen(createDidOpenEvent(file, "EmbeddedMontiArc"));
        assertFalse(DiagnosticsLog.getFindings().isEmpty());
    }

}