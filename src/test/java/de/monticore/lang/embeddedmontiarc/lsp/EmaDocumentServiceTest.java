package de.monticore.lang.embeddedmontiarc.lsp;

import de.se_rwth.commons.logging.DiagnosticsLog;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.*;
import java.util.concurrent.ExecutionException;

import static org.junit.jupiter.api.Assertions.assertTrue;

class EmaDocumentServiceTest {

    @BeforeAll
    public static void setup(){
        DiagnosticsLog.init();
    }

    @Test
    public void testValid() throws InterruptedException, ExecutionException, IOException {
        EmaDocumentService documentService = new EmaDocumentService();
        documentService.doParse(new StringReader("package a; component Abc{}"));
        assertTrue(DiagnosticsLog.getFindings().isEmpty());
    }

    @Test
    public void testInvalid() throws InterruptedException, ExecutionException, IOException {
        EmaDocumentService documentService = new EmaDocumentService();
        documentService.doParse(new StringReader("package a; compoent Abc{}"));
        assertTrue(!DiagnosticsLog.getFindings().isEmpty());
    }

}