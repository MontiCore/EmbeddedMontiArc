package de.monticore.lang.monticar.borealis.runtime.language.converters.common;

import de.se_rwth.commons.SourcePosition;

public class SourcePositions {
    public final SourcePosition start;
    public final SourcePosition end;

    public SourcePositions(SourcePosition start, SourcePosition end) {
        this.start = start;
        this.end = end;
    }
}
