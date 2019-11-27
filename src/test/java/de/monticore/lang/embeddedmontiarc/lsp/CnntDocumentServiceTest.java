package de.monticore.lang.embeddedmontiarc.lsp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarccnntrainlang.lsp.CnntDocumentService;
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

// TODO write tests

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
}