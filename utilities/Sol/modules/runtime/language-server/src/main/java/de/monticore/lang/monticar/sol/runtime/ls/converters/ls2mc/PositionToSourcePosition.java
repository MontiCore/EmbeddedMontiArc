/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.converters.ls2mc;

import com.google.inject.Singleton;
import de.se_rwth.commons.SourcePosition;
import org.eclipse.lsp4j.Position;

import java.util.function.Function;

@Singleton
public class PositionToSourcePosition implements Function<Position, SourcePosition> {
    protected PositionToSourcePosition() {}

    @Override
    public SourcePosition apply(Position position) {
        return new SourcePosition(position.getLine() + 1, position.getCharacter());
    }
}
