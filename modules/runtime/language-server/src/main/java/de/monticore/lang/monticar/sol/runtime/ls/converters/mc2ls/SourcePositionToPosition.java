/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.converters.mc2ls;

import com.google.inject.Singleton;
import de.se_rwth.commons.SourcePosition;
import org.eclipse.lsp4j.Position;

import java.util.function.Function;

@Singleton
public class SourcePositionToPosition implements Function<SourcePosition, Position> {
    protected SourcePositionToPosition() {}

    @Override
    public Position apply(SourcePosition sourcePosition) {
        return new Position(sourcePosition.getLine() - 1, sourcePosition.getColumn());
    }
}
