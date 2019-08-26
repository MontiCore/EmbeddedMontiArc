/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ranges._ast;

import java.util.List;
import java.util.stream.Collectors;

/**
 * Created by michaelvonwenckstern on 11.02.17.
 */
public class ASTRanges extends ASTRangesTOP {
    public ASTRanges() {
        super();
    }

    public ASTRanges(List<ASTRange> ranges) {
        super(ranges);
    }

    @Override
    public String toString() {
        return String.format("[%s]",
                getRangeList().stream()
                        .map(ASTRange::toString).collect(Collectors.joining(", ")));
    }
}
