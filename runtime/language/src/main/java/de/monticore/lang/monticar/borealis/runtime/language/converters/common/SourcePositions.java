package de.monticore.lang.monticar.borealis.runtime.language.converters.common;

import de.se_rwth.commons.SourcePosition;

public class SourcePositions {
    public final SourcePosition start;
    public final SourcePosition end;

    public SourcePositions(SourcePosition start, SourcePosition end) {
        this.start = start;
        this.end = end;
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof SourcePositions && this.equals((SourcePositions) o);
    }

    protected boolean equals(SourcePositions sourcePositions) {
        return this.start.equals(sourcePositions.start) && this.end.equals(sourcePositions.end);
    }
}
