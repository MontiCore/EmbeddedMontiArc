/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar;

import de.monticore.lang.monticar.ts.MontiCarTypeSymbol;
import de.monticore.symboltable.MutableScope;
import org.jscience.mathematics.number.Rational;

public final class Utils {

    private Utils() {
    }

    public static void addBuiltInTypes(MutableScope scope) {
        String[] builtInTypes = new String[]{
                "Q",
                "B",
                "C",
                "Z",
                "RangesType",
                "RangeType",
                "NumberWithUnitResolution",
                "NumberWithUnitTypeArgument",
                "AssignmentType",
                "CommonMatrixType"
        };
        for (String typeName : builtInTypes) {
            MontiCarTypeSymbol s = new MontiCarTypeSymbol(typeName);
            s.setPackageName("java.lang");
            scope.add(s);
        }
    }

    public static String getRationalString(Rational rational) {
        String valueString = rational.toString();
        if (rational.getDivisor().toString().equals("1")) {
            valueString = rational.getDividend().toString();
        }
        return valueString;
    }
}
