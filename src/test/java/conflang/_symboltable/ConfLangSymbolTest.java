package conflang._symboltable;

import conflang.AbstractTest;
import conflang.ConfLangMill;
import sun.rmi.runtime.Log;

public class ConfLangSymbolTest extends AbstractTest {

    private ConfLangSymbol confLangSymbol;

    public static void init() {
        Log.enableFailQuick(false);
    }

    public void setUp() {
        LogStub.init();
        Log.getFindings().clear();
        confLangSymbol = ConfLangMill.confLangSymbolBuilder().build();
    }
}