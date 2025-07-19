package de.monticore.lang.monticar.cnnarch.generator.training;

public enum RlAlgorithm {

    DQN("dqn"),
    DDPG("ddpg"),
    TD3("td3");

    String algorithm;

    RlAlgorithm(String algorithm) {
        this.algorithm = algorithm;
    }

    public static RlAlgorithm rlAlgorithm(String algorithm) {
        for (RlAlgorithm al : values()) {
            if (al.algorithm.equals(algorithm)) {
                return al;
            }
        }
        throw new IllegalArgumentException(String.valueOf(algorithm));
    }

    public String getAlgorithm() {
        return algorithm;
    }
}