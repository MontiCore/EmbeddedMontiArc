/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.converters.ls2mc;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.runtime.ls.converters.common.SourcePositions;
import de.se_rwth.commons.SourcePosition;
import org.eclipse.lsp4j.Range;

import java.util.function.Function;

@Singleton
public class RangeToSourcePositions implements Function<Range, SourcePositions> {
    protected final PositionToSourcePosition p2sp;

    @Inject
    protected RangeToSourcePositions(PositionToSourcePosition p2sp) {
        this.p2sp = p2sp;
    }

    @Override
    public SourcePositions apply(Range range) {
        SourcePosition start = this.p2sp.apply(range.getStart());
        SourcePosition end = this.p2sp.apply(range.getEnd());

        return new SourcePositions(start, end);
    }
}
