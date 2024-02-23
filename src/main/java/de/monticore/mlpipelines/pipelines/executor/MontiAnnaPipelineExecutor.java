package de.monticore.mlpipelines.pipelines.executor;

import de.monticore.mlpipelines.tracking.helper.ASTConfLangHelper;
import de.monticore.mlpipelines.tracking.tracker.MultiBackendTracker;

public class MontiAnnaPipelineExecutor extends PipelineExecutor {

    @Override
    public void execute() {
        // Initialize run tracker
        MultiBackendTracker runTracker = getMontiAnnaContext().getTrackerFactory().createTracker();
        trainPipeline.setRunTracker(runTracker);

        for(int index = 0; index < networkExecutionConfigs.size(); index++) {
            this.applyBasicConfiguration(index);

            runTracker.startNewRun();
            runTracker.logTag("Mode", "Single Run");
            runTracker.logTag("Root Model", montiAnnaContext.getRootModelName());
            runTracker.logTag("Network", trainPipeline.getNetworkName());
            runTracker.getArtifactHandler().setArtifacts(emadlFiles).setPath("emadl-Files").addExtension(".txt").log();
            runTracker.getArtifactHandler().setPlaintext(trainPipeline.getPrettyPrintedNetwork()).setFileName("network.txt").log();
            runTracker.logParams(ASTConfLangHelper.getParametersFromConfiguration(trainPipeline.getTrainingConfiguration()));

            // Execute pipeline
            trainPipeline.execute();

            // End current run
            runTracker.endRun();
        }
        runTracker.close();
    }
}
