package de.monticore.mlpipelines.workflow;

import de.monticore.lang.monticar.cnnarch.generator.training.LearningMethod;
import de.monticore.mlpipelines.automl.AutoMLPipeline;
import de.monticore.mlpipelines.configuration.MontiAnnaContext;
import de.monticore.mlpipelines.pipelines.PythonPipeline;

public class AutoMLPipelineWorkflow extends AbstractWorkflow {
    public AutoMLPipelineWorkflow(final MontiAnnaContext applicationContext) {
        super(applicationContext);
    }

    public void createPipeline(final LearningMethod learningMethod) {
        final PythonPipeline pythonPipeline = new PythonPipeline(learningMethod);
        pythonPipeline.setMontiAnnaGenerator(this.montiAnnaGenerator);

        final AutoMLPipeline autoMLPipeline = new AutoMLPipeline(learningMethod);
        autoMLPipeline.setTrainPipeline(pythonPipeline);

        this.setPipeline(autoMLPipeline);
    }
}
