package de.monticore.lang.monticar.cnnarch.generator.training;

public enum LearningMethod {

    SUPERVISED("supervised"),
    REINFORCEMENT("reinforcement"),
    GAN("gan"),
    VAE("vae");

    String method;

    LearningMethod(String method) {
        this.method = method;
    }

    public static LearningMethod learningMethod(String learningMethod) {
        for (LearningMethod lm : values()) {
            if (lm.method.equals(learningMethod)) {
                return lm;
            }
        }
        throw new IllegalArgumentException(String.valueOf(learningMethod));
    }
}