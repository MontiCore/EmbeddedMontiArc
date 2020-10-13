package de.monticore.lang.embeddedmontiarc.lsp;

import de.monticore.lang.embeddedmontiarc.cnnarchlang.lsp.CnnaDocumentService;
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

class CnnaDocumentServiceTest extends AbstractTextDocumentServiceTest {

    public static final String BASE_PATH = "src/test/resources/cnna";

    private CnnaDocumentService getDocumentService(String basePath) throws IOException {
        CnnaDocumentService res = new CnnaDocumentService(new ModelFileCache(Collections.singleton("cnna")));
        res.setClient(getMockClient());
        res.setModelFileCache(new ModelFileCache(Paths.get(basePath).toAbsolutePath() , Collections.singleton(".cnna")));
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
        CnnaDocumentService documentService = getDocumentService(BASE_PATH + "/valid");
        DiagnosticsLog.setLogToStdout(true);
        documentService.doParse(new StringReader("architecture SimplerArchitecture{def input Q(0:1)^{1,28,28} image    def output Q(0:1)^{10} image2    image ->   FullyConnected(units=10) -> Softmax() ->     image2;}"));
        assertTrue(DiagnosticsLog.getFindings().isEmpty());
    }

    @Test
    public void testInvalidSyntax() throws InterruptedException, ExecutionException, IOException {
        CnnaDocumentService documentService = getDocumentService(BASE_PATH + "/invalid");
        documentService.doParse(new StringReader("architeture valid{}"));
        assertFalse(DiagnosticsLog.getFindings().isEmpty());
    }

    @Test
    public void testValidDidOpenEvent() throws IOException {
        CnnaDocumentService documentService = getDocumentService(BASE_PATH + "/valid");

        File file = new File("src/test/resources/cnna/valid/SimpleArchitecture.cnna");
        // File file = new File("src/test/resources/cnna/valid/Alexnet_alt.cnna");
        documentService.didOpen(createDidOpenEvent(file, "CNNArchLang"));
        assertTrue(DiagnosticsLog.getFindings().isEmpty());
    }

    @Test
    public void testInvalidDidOpenEvent() throws IOException {
        CnnaDocumentService documentService = getDocumentService(BASE_PATH + "/invalid");
        File file = new File("src/test/resources/cnna/invalid/SimpleArchitecture.cnna");
        documentService.didOpen(createDidOpenEvent(file, "CNNArchLang"));
        assertFalse(DiagnosticsLog.getFindings().isEmpty());
    }
}
