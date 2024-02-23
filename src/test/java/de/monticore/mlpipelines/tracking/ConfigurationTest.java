package de.monticore.mlpipelines.tracking;

import de.se_rwth.commons.logging.Log;
import de.se_rwth.commons.logging.LogStub;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

public class ConfigurationTest {

    @BeforeAll
    public static void init() {
        LogStub.init();
    }

    @BeforeEach
    public void clearFindings() {
        Log.getFindings().clear();
    }

    @Test
    public void testNoConfiguration() {
        final TrackerFactory trackerFactory = new TrackerFactory(null);
        trackerFactory.createTracker();
        Assertions.assertEquals(0, Log.getFindings().size());
    }

    @Test
    public void testValidConfiguration() {
        final TrackerFactory trackerFactory = new TrackerFactory("src/test/resources/tracking/configuration/valid/validConfiguration.conf");
        trackerFactory.createTracker();
        Assertions.assertEquals(0, Log.getFindings().size());
    }

    @ParameterizedTest
    @ValueSource(strings = {
            "src/test/resources/tracking/configuration/invalid/invalidBlacklist.conf",
            "src/test/resources/tracking/configuration/invalid/invalidTags1.conf",
            "src/test/resources/tracking/configuration/invalid/invalidTags2.conf",
            "src/test/resources/tracking/configuration/invalid/invalidTags3.conf"})
    public void testInvalidConfigurations(String path) {
        final TrackerFactory trackerFactory = new TrackerFactory(path);
        trackerFactory.createTracker();
        Assertions.assertEquals(1, Log.getFindings().size());
    }

}
