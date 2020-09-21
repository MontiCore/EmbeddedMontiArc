/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.helper;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Function;

/**
 */
public class IndentPrinterParameter {
    protected ArrayList<Object> params = new ArrayList<>();
    protected ArrayList<String> groups;

    protected IndentPrinterParameter(ArrayList<String> groups, Object... params) {
        this.groups = groups;
        for (Object p : params) {
            this.params.add(p);
        }
    }

    @Override
    public String toString() {
        return (new IndentPrinterHandler(groups, params, new LinkedHashMap<>()).toString());
    }

    public <A> IndentPrinterHandler handle(Class<? extends A> clazz, Function<A, String> handle1) {
        Map<Class, Function<Object, String>> map = new LinkedHashMap<>();
        map.put(clazz, (Function<Object, String>) handle1);
        return new IndentPrinterHandler(groups, params, map);
    }

//    public <A, B> IndentPrinterHandler handle(Supplier<A> handle1, Supplier<B> handle2) {
//      return new IndentPrinterHandler(groups, params, handle1, handle2);
//    }
}
