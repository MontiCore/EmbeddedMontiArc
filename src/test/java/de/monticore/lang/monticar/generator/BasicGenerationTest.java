/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.LogConfig;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;

public abstract class BasicGenerationTest {

    @BeforeClass
    public static void initLog() {
        LogConfig.init();
        LogConfig.enableFailQuick(false);
    }

    @Before
    public void clearFindings() {
        LogConfig.getFindings().clear();
    }

    @AfterClass
    public static void resetLog() {
        LogConfig.getFindings().clear();
        LogConfig.enableFailQuick(true);
    }

}
