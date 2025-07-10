/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.helper;

import java.util.ArrayList;
import java.util.LinkedHashMap;

/**
 */
public class IndentPrinterGroup {
    protected ArrayList<String> groups = new ArrayList<>();

    protected IndentPrinterGroup(String... groups) {
        for (String s : groups) {
            this.groups.add(s);
        }
    }

    @Override
    public String toString() {
        return (new IndentPrinterHandler(groups, new ArrayList<>(), new LinkedHashMap<>())).toString();
    }


    // 26 methods overloaded here, b/c
    // public <T...> IndentPrinterParameter setParams(T... params)
    // is not possible in Java


    public <A> IndentPrinterParameter params(A param1) {
        return new IndentPrinterParameter(groups, param1);
    }

    public <A, B> IndentPrinterParameter params(A param1, B param2) {
        return new IndentPrinterParameter(groups, param1, param2);
    }

    public <A, B, C> IndentPrinterParameter params(A param1, B param2, C param3) {
        return new IndentPrinterParameter(groups, param1, param2, param3);
    }

    public <A, B, C, D> IndentPrinterParameter params(A param1, B param2, C param3, D param4) {
        return new IndentPrinterParameter(groups, param1, param2, param3, param4);
    }

    public <A, B, C, D, E> IndentPrinterParameter params(A param1, B param2, C param3, D param4, E param5) {
        return new IndentPrinterParameter(groups, param1, param2, param3, param4, param5);
    }

    public <A, B, C, D, E, F> IndentPrinterParameter params(A param1, B param2, C param3, D param4, E param5, F param6) {
        return new IndentPrinterParameter(groups, param1, param2, param3, param4, param5, param6);
    }

    public <A, B, C, D, E, F, G> IndentPrinterParameter params(A param1, B param2, C param3, D param4, E param5, F param6, G param7) {
        return new IndentPrinterParameter(groups, param1, param2, param3, param4, param5, param6, param7);
    }

    public <A, B, C, D, E, F, G, H> IndentPrinterParameter params(A param1, B param2, C param3, D param4, E param5, F param6, G param7, H param8) {
        return new IndentPrinterParameter(groups, param1, param2, param3, param4, param5, param6, param7, param8);
    }

    // ...


}
