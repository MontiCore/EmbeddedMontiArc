package de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.critic;

public class CriticNetworkGenerationException extends RuntimeException {
    public CriticNetworkGenerationException(String s) {
        super("Generation of critic network failed: " + s);
    }
}
