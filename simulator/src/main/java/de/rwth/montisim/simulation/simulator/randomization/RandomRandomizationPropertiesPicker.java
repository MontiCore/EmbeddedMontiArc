package de.rwth.montisim.simulation.simulator.randomization;

import java.util.Optional;
import java.util.Random;
import java.util.Vector;
import java.util.concurrent.ThreadLocalRandom;
import java.util.stream.Collectors;

public class RandomRandomizationPropertiesPicker {

    /**
     * @param availableRandomizationProperties
     * @return Returns a randomly picked RandomisationProperties from a Vector, if the Vector is not empty, otherwise Empty Strategy Properties
     */
    public static RandomizationProperties pickRandomizationProperties(Vector<RandomizationProperties> availableRandomizationProperties) throws Exception {
        if (availableRandomizationProperties.size() == 0)
            return new EmptyStrategyProperties();

        Random random = ThreadLocalRandom.current();

        // Simple Verification
        // get strategy from type
        Vector<Strategy> randomStrategyVector = availableRandomizationProperties.stream().map(properties -> Strategy.getByPropertiesClass(properties))
                .filter(opt -> opt.isPresent())
                .map(opt -> opt.get())
                .collect(Collectors.toCollection(Vector::new));
        if (availableRandomizationProperties.size() != randomStrategyVector.size()) {
            throw new Exception("At least one of the types of strategies for the random strategy selection was not recognized.");
        }
        // check if strategy is selectables
        randomStrategyVector = randomStrategyVector.stream()
                .filter(strategy -> strategy.randomPickable)
                .collect(Collectors.toCollection(Vector::new));
        if (availableRandomizationProperties.size() != randomStrategyVector.size()) {
            throw new Exception("At least one of the strategies for the random strategy selection is not selectable in this context.");
        }

        // Select random RandomizationProperties
        return availableRandomizationProperties.get(random.nextInt(availableRandomizationProperties.size()));
    }

}
