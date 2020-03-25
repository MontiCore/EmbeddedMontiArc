/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.helper;

import com.google.common.collect.Lists;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.function.Function;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 */
public class IndentPrinterHandler {
    // \{(\d)(?:\s*:\s*'([^']+)')?(?:\s*:\s*'([^']+)')?(?:\s*:\s*'([^']+)')?\}
    // this finds (and groups it):
    // {0}
    // {1 : ','}
    // {2 : 'before' : 'endVal'}
    // {3 : 'before' : 'middle' : 'endVal'}
    protected final static Pattern pattern = Pattern.compile("\\{(\\d)(?:\\s*:\\s*'([^']+)')?(?:\\s*:\\s*'([^']+)')?(?:\\s*:\\s*'([^']+)')?\\}");
    protected Map<Class, Function<Object, String>> functions;
    protected ArrayList<Object> params;
    protected ArrayList<String> groups;

    protected IndentPrinterHandler(ArrayList<String> groups, ArrayList<Object> params, Map<Class, Function<Object, String>> functions) {
        this.groups = groups;
        this.params = params;
        this.functions = functions;
    }

    public void toStringProcessing() {
        for (int i = 0; i < groups.size(); i++) {
            int j = 0;
            Matcher m = pattern.matcher(groups.get(i));
            while (m.find()) {
                if (j > 0) {
                    String left = groups.get(i).substring(0, m.start());
                    String right = groups.get(i).substring(m.start());
                    groups.remove(i);
                    groups.add(i, left);
                    groups.add(i + 1, right);
                }
                j++;
            }
        }
    }

    @Override
    public String toString() {
        toStringProcessing();
        IndentPrinter ip = new IndentPrinter();
        for (String g : groups) {
            // only one variable place holder per group
            Matcher m = pattern.matcher(g);
            if (m.find()) {
                int pos = Integer.parseInt(m.group(1));
                if (pos < params.size()) {
                    Object param = params.get(pos);
                    Function<Object, String> fnc = o -> o.toString();
                    if (functions.containsKey(param.getClass()))
                        fnc = functions.get(param.getClass());
                    if (param instanceof Collection) {
                        printCollection(param, m, ip, g);
                    } else if (param instanceof Optional) {
                        fnc = printOptional(param, m, ip, g, fnc);
                    } else
                        ip.print(g.replace(m.group(0), fnc.apply(param)));
                } else {
                    Log.warn("Index " + pos +
                            " is out of range. No object at this position is specified, please startVal at 0. The string " +
                            m.group(0) + " will not replaced with empty string.");
                    ip.print(g.replace(m.group(0), ""));
                }
            }
        }
        return ip.getContent();
    }

    public Function<Object, String> printOptional(Object param, Matcher m, IndentPrinter ip, String g, Function<Object, String> fnc) {
        if (((Optional) param).isPresent()) {
            if (functions.containsKey(((Optional) param).get().getClass())) {
                fnc = functions.get(param.getClass());
            }
            ip.print(g.replace(m.group(0), fnc.apply(((Optional) param).get())));
        }
        return fnc;
    }

    public void printCollection(Object param, Matcher m, IndentPrinter ip, String g) {
        List c = Lists.newArrayList((Collection) param);
        if (m.group(2) != null && m.group(3) != null &&
                !c.isEmpty()) {
            ip.print(m.group(2));
        }
        String delim = "";
        if (m.group(2) != null && m.group(3) == null) {
            delim = m.group(2);
        } else if (m.group(2) != null && m.group(3) != null && m.group(4) != null) {
            delim = m.group(3);
        }
        for (int i = 0; i < c.size(); i++) {
            if (functions.containsKey(c.get(i).getClass())) {
                ip.print(g.replace(m.group(0), functions.get(c.get(i).getClass()).apply(c.get(i))));
            } else {
                ip.print(g.replace(m.group(0), c.get(i).toString()));
            }
            if (i != c.size() - 1) {
                ip.print(delim);
            }
        }
        if (m.group(2) != null && m.group(3) != null && m.group(4) == null && !c.isEmpty()) {
            ip.print(m.group(3));
        } else if (m.group(2) != null && m.group(3) != null && m.group(4) != null && !c.isEmpty()) {
            ip.print(m.group(4));
        }
    }
}
