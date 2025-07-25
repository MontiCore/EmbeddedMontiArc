/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.grammar.symboltable;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class HeritageSet<T> {
    protected final List<T> inclusions;
    protected final List<T> exclusions;

    public HeritageSet() {
        this.inclusions = new ArrayList<>();
        this.exclusions = new ArrayList<>();
    }

    public void include(T value) {
        this.inclusions.add(value);
    }

    public void exclude(T value) {
        this.exclusions.add(value);
    }

    public List<T> getInclusions() {
        return Collections.unmodifiableList(this.inclusions);
    }

    public List<T> getExclusions() {
        return Collections.unmodifiableList(this.exclusions);
    }
}
