package de.rwth.montisim.simulation.simulator.randomization;

import java.util.Optional;

public enum Strategy {

    EmptyStrategy(EmptyStrategyProperties.class, false),
    BasicDrivingStrategy(BasicDrivingStrategyProperties.class, true),
    IntersectionStrategy(IntersectionStrategyProperties.class, true),
    PlatooningStrategy(PlatooningStrategyProperties.class, true);

    public final Class<? extends RandomizationProperties> propertiesClass;
    public final boolean randomPickable; // whether or not the RandomStrategyProperties can select this Strategy as a random strategy

    private Strategy(Class<? extends RandomizationProperties> propertiesClass, boolean randomPickable) {
        this.propertiesClass = propertiesClass;
        this.randomPickable = randomPickable;
    }

    public static Optional<Strategy> getByPropertiesClass(Class<? extends RandomizationProperties> propertiesClass) {
        for (Strategy strategy : values()) {
            if (strategy.propertiesClass == propertiesClass)
                return Optional.of(strategy);
        }
        return Optional.empty();
    }

    public static Optional<Strategy> getByPropertiesClass(RandomizationProperties properties) {
        return getByPropertiesClass(properties.getClass());
    }
}
