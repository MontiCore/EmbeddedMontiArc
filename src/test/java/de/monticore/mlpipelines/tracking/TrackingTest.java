package de.monticore.mlpipelines.tracking;

import de.monticore.mlpipelines.tracking.helper.SimpleRegexMatcher;
import de.monticore.mlpipelines.tracking.tracker.MultiBackendTracker;
import de.se_rwth.commons.logging.Log;
import de.se_rwth.commons.logging.LogStub;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TrackingTest {

    @BeforeAll
    public static void init() {
        LogStub.init();
    }

    @BeforeEach
    public void clearFindings() {
        Log.getFindings().clear();
    }

    private MultiBackendTracker createMultiBackendTracker(RunTrackerStub stub) {
        return new MultiBackendTracker(Collections.singletonList(stub),
                new ArrayList<>(),
                new HashMap<>(),
                new SimpleRegexMatcher());
    }

    @Test
    public void testTagLogging(){
        RunTrackerStub stub = new RunTrackerStub();
        MultiBackendTracker tracker = createMultiBackendTracker(stub);
        Map<String, String> stubTags = stub.getTags();

        // Insert & read one tag
        tracker.logTag("key", "value");
        Assertions.assertEquals(1, stubTags.size());
        Assertions.assertEquals("value", stubTags.get("key"));

        // Insert & read multiple tags
        Map<String, String> tagsToAdd = new HashMap<>();
        tagsToAdd.put("key1", "value1");
        tagsToAdd.put("key2", "value2");
        tracker.logTags(tagsToAdd);
        Assertions.assertEquals(3, stubTags.size());
        Assertions.assertEquals("value1", stubTags.get("key1"));
        Assertions.assertEquals("value2", stubTags.get("key2"));
    }

    @Test
    public void testParameterLogging() {
        RunTrackerStub stub = new RunTrackerStub();
        MultiBackendTracker tracker = createMultiBackendTracker(stub);
        Map<String, String> stubParams = stub.getParams();

        // Insert & read one param
        tracker.logParam("key", "value");
        Assertions.assertEquals(1, stubParams.size());
        Assertions.assertEquals("value", stubParams.get("key"));

        // Insert & read multiple params
        Map<String, String> paramsToAdd = new HashMap<>();
        paramsToAdd.put("key1", "value1");
        paramsToAdd.put("key2", "value2");
        tracker.logParams(paramsToAdd);
        Assertions.assertEquals(3, stubParams.size());
        Assertions.assertEquals("value1", stubParams.get("key1"));
        Assertions.assertEquals("value2", stubParams.get("key2"));
    }

    @Test
    public void testMetricLogging() {
        RunTrackerStub stub = new RunTrackerStub();
        MultiBackendTracker tracker = createMultiBackendTracker(stub);
        Map<String, Double> stubMetrics = stub.getMetrics();

        // Insert & read multiple metrics
        tracker.logMetric("accuracy", 1.0, 0, 0);
        Assertions.assertEquals(1, stubMetrics.size());
        Assertions.assertEquals(1.0, stubMetrics.get("accuracy"));

        tracker.logMetric("loss", 0d);
        Assertions.assertEquals(2, stubMetrics.size());
        Assertions.assertEquals(0d, stubMetrics.get("loss"));
    }

    @Test
    public void testArtifactLogging() throws IOException {
        RunTrackerStub stub = new RunTrackerStub();
        MultiBackendTracker tracker = createMultiBackendTracker(stub);
        Map<File, String> stubArtifacts = stub.getArtifacts();

        // Simple file logging
        File testFile = new File("src/test/resources/tracking/testArtifact.emadl");
        tracker.logArtifact(testFile, "test/path");
        Assertions.assertEquals(1, stubArtifacts.size());
        Assertions.assertEquals("test/path", stubArtifacts.get(testFile));
        stubArtifacts.clear();

        // Test file logging builder
        // Test Case 1: Simple artifact
        tracker.getArtifactHandler().setArtifact(testFile).setFileName("newFileName").setPath("new/path").log();
        Assertions.assertEquals(1, stubArtifacts.size());
        Entry<File, String> entry = stubArtifacts.entrySet().stream().findFirst().get();
        Assertions.assertEquals("newFileName", entry.getKey().getName());
        Assertions.assertEquals("new/path", entry.getValue());
        stubArtifacts.clear();

        // Test Case 2: Plaintext artifact
        tracker.getArtifactHandler().setPlaintext("Hello\nWorld").setFileName("plaintextArtifact").setPath("new/plaintext/path").log();
        Assertions.assertEquals(1, stubArtifacts.size());
        entry = stubArtifacts.entrySet().stream().findFirst().get();
        Assertions.assertEquals("plaintextArtifact", entry.getKey().getName());
        Assertions.assertEquals("new/plaintext/path", entry.getValue());
        stubArtifacts.clear();

        // Test Case 3: Multiple artifacts
        tracker.getArtifactHandler().setArtifacts(Collections.singletonList(testFile)).addExtension(".txt").setPath("new/multiple/path").log();
        Assertions.assertEquals(1, stubArtifacts.size());
        entry = stubArtifacts.entrySet().stream().findFirst().get();
        Assertions.assertEquals("testArtifact.emadl.txt", entry.getKey().getName());
        Assertions.assertEquals("new/multiple/path", entry.getValue());
        stubArtifacts.clear();
    }

    @Test
    public void testBlacklist() {
        RunTrackerStub stub = new RunTrackerStub();
        MultiBackendTracker tracker = new MultiBackendTracker(Collections.singletonList(stub),
                new ArrayList<>(),
                new HashMap<>(),
                new SimpleRegexMatcher(Arrays.asList("context", "optimizer.*")));
        Map<String, String> stubParams = stub.getParams();

        // Test Case 1: Blacklisted keys
        tracker.logParam("context", "cpu");
        tracker.logParam("optimizer.learning_rate", "0.001");
        tracker.logParam("optimizer.beta1", "0.9");
        Assertions.assertEquals(0, stubParams.size());

        // Test Case 2: Non-blacklisted keys
        tracker.logParam("num_epoch", "10");
        tracker.logParam("batch_size", "64");
        tracker.logParam("optimizer", "adam");
        Assertions.assertEquals(3, stubParams.size());
    }

}
