/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.converters.mc2ls;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.runtime.ls.converters.common.SourcePositions;
import org.eclipse.lsp4j.Position;
import org.eclipse.lsp4j.Range;

import java.util.function.Function;

@Singleton
public class SourcePositionsToRange implements Function<SourcePositions, Range> {
    protected final SourcePositionToPosition sp2p;

    @Inject
    protected SourcePositionsToRange(SourcePositionToPosition sp2p) {
        this.sp2p = sp2p;
    }

    @Override
    public Range apply(SourcePositions sourcePositions) {
        Position start = this.sp2p.apply(sourcePositions.start);
        Position end = this.sp2p.apply(sourcePositions.end);

        return new Range(start, end);
    }
}
