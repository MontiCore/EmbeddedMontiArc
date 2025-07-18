package schemalang.validation;

import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;

public class TypeCompatibilityTest {

    @BeforeClass
    public static void init() {
        Log.init();
        Log.enableFailQuick(false);
    }

    @Before
    public void before() {
        Log.clearFindings();
    }
}