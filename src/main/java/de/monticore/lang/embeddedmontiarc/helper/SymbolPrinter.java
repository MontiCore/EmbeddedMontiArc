/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.monticar.ValueSymbol;
import de.monticore.lang.monticar.helper.IndentPrinter;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.types.TypeSymbol;
import de.monticore.symboltable.types.references.ActualTypeArgument;
import de.monticore.symboltable.types.references.TypeReference;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Created by Michael von Wenckstern on 20.05.2016.
 * class for pretty printing symbols, this class does not use the AST
 */
public class SymbolPrinter {


    /**
     * help function for nested type arguments such as List<NewType<String, List<String>>>
     */
    public static String printTypeParameters(ActualTypeArgument arg) {
        // String ret = arg.getType().getReferencedSymbol().getFullName();
        String ret = arg.getType().getName();
        if (arg.getType().getActualTypeArguments() != null && !arg.getType().getActualTypeArguments().isEmpty()) {
            ret += "<" + arg.getType().getActualTypeArguments().stream().
                    map(a -> printWildCardPrefix(a) + printTypeParameters(a) + printArrayDimensions(a)).collect(Collectors.joining(",")) + ">";
        }
        return ret;
    }

    protected static String printWildCardPrefix(ActualTypeArgument a) {
        if (a.isLowerBound()) {
            return "? super ";
        } else if (a.isUpperBound()) {
            return "? extends ";
        }
        return "";
    }

    protected static String printArrayDimensions(ActualTypeArgument a) {
        if (a.getType() instanceof MCTypeReference) {
            int dim = ((MCTypeReference) a.getType()).getDimension();
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < dim; i++) {
                sb.append("[]");
            }
            return sb.toString();
        }
        return "";
    }

    public static String printTypeParameters(List<ActualTypeArgument> arg) {
        if (arg == null || arg.isEmpty())
            return "";
        return "<" + arg.stream().map(a -> printWildCardPrefix(a) + printTypeParameters(a) + printArrayDimensions(a)).
                collect(Collectors.joining(",")) + ">";
    }

    /**
     * help function for nested type arguments such as List<NewType<String, List<String>>>
     */
    public static String printFormalTypeParameters(MCTypeSymbol arg) {
        String ret = arg.getName();

        if (!arg.getSuperTypes().isEmpty()) {
            ret += " extends " + arg.getSuperTypes().stream()
                    .map(t -> t.getReferencedSymbol().getFullName() + printTypeParameters(t.getActualTypeArguments()))
                    .collect(Collectors.joining("&"));
        }

        if (arg.getFormalTypeParameters() != null && !arg.getFormalTypeParameters().isEmpty()) {
            ret += "<" + arg.getFormalTypeParameters().stream().
                    map(a -> printFormalTypeParameters(a)).collect(Collectors.joining(",")) + ">";
        }
        return ret;
    }

    /**
     * @return string representation of the type parameters associated with this port.
     */
    public static String printFormalTypeParameters(List<MCTypeSymbol> arg) {
        if (arg.isEmpty())
            return "";
        return "<" + arg.stream()
                .map(a -> printFormalTypeParameters(a))
                .collect(Collectors.joining(",")) + ">";
    }

    public static void printPort(EMAPortSymbol port, IndentPrinter ip) {
        if (port.isIncoming()) {
            ip.print("in ");
        } else {
            ip.print("out ");
        }
        ip.print(port.getTypeReference().getName());
        ip.print(printTypeParameters(port.getTypeReference().getActualTypeArguments()));
        ip.print(" ");
        ip.print(port.getName());

        printTags(port, ip);
    }

    public static String printPort(EMAPortSymbol port) {
        IndentPrinter ip = new IndentPrinter();
        printPort(port, ip);
        return ip.getContent();
    }

    public static void printConnector(EMAConnectorSymbol con, IndentPrinter ip) {
        ip.print(con.getSource());
        ip.print(" -> ");
        ip.print(con.getTarget());

        printTags(con, ip);
    }

    public static String printConnector(EMAConnectorSymbol con) {
        IndentPrinter ip = new IndentPrinter();
        printConnector(con, ip);
        return ip.getContent();
    }

    public static String printConfigArguments(List<ValueSymbol<TypeReference<TypeSymbol>>> config) {
        if (config.isEmpty())
            return "";
        return "(" + config.stream().map(a -> a.getValue()).collect(Collectors.joining(",")) + ")";
    }

    public static void printEMAComponentInstantiation(EMAComponentInstantiationSymbol inst, IndentPrinter ip) {
        ip.print(inst.getComponentType().getName());
        ip.print(printTypeParameters(inst.getComponentType().getActualTypeArguments()));
        ip.print(printConfigArguments(inst.getConfigArguments()));
        ip.print(" ");
        ip.print(inst.getName());
    }

    public static String printEMAComponentInstantiation(EMAComponentInstantiationSymbol inst) {
        IndentPrinter ip = new IndentPrinter();
        printEMAComponentInstantiation(inst, ip);
        return ip.getContent();
    }

    public static void printPorts(Collection<? extends EMAPortSymbol> ports, IndentPrinter ip) {
        if (!ports.isEmpty()) {
            ip.println("ports");
            ip.indent();
            int i = 0;
            int s = ports.size();
            for (EMAPortSymbol p : ports) {
                printPort(p, ip);
                if (i == s - 1) {
                    ip.println(";");
                } else {
                    ip.println(",");
                }
                i++;
            }
            ip.unindent();
        }
    }

    public static void printComponent(EMAComponentSymbol cmp, IndentPrinter ip, boolean skipPackageImport) {
        printPackageInfo(cmp, ip, skipPackageImport);
        ip.print("component " + cmp.getName());
        if (cmp.hasFormalTypeParameters()) {
            ip.print(printFormalTypeParameters(cmp.getFormalTypeParameters()));
        }
        if (cmp.hasConfigParameters()) {
            ip.print("(");
            ip.print(cmp.getConfigParameters().stream().map(a -> a.getType().getName() + " " + a.getName()).
                    collect(Collectors.joining(",")));
            ip.print(")");
        }
        ip.println(" {");
        printTags(cmp, ip);

        ip.indent();

        printPorts(cmp.getPortsList(), ip);

        cmp.getSubComponents().stream().forEachOrdered(a -> {
            ip.print("component ");
            printEMAComponentInstantiation(a, ip);
            ip.println(";");
        });

        cmp.getInnerComponents().stream().forEachOrdered(a -> printComponent(a, ip, true));

        cmp.getConnectors().stream().forEachOrdered(a -> {
            ip.print("connect ");
            printConnector(a, ip);
            ip.println(";");
        });

        ip.unindent();
        ip.println("}");
    }

    public static String printComponent(EMAComponentSymbol cmp) {
        IndentPrinter ip = new IndentPrinter();
        printComponent(cmp, ip, false);
        return ip.getContent();
    }

    public static void printTags(Symbol hasTags, IndentPrinter ip) {
        // needs to be changed so that this method can be overwritten --> remove static
//        if (!hasTags.getTags().isEmpty()) {
//            ip.println("/* tags: ");
//            ip.indent();
//            ip.indent();
//            ip.print(hasTags.getTags().stream().map(t -> t.toString()).
//                    collect(Collectors.joining("\n")));
//            ip.print("  */");
//            ip.unindent();
//            ip.unindent();
//            ip.println();
//        }
    }

    public static void printPackageInfo(EMAComponentSymbol cmp, IndentPrinter ip, boolean skipPackageImport) {
        if (!skipPackageImport) {
            if (cmp.getPackageName() != null &&
                    !cmp.getPackageName().isEmpty()) {
                ip.print("package ");
                ip.print(cmp.getPackageName());
                ip.println(";");
            }
            if (cmp.getImports() != null) {
                cmp.getImports().stream().forEachOrdered(a -> ip.println("import " + a.getStatement() + (a.isStar() ? ".*" : "") + ";"));
            }
        }
    }

    public static void printEMAComponentInstance(EMAComponentInstanceSymbol inst, IndentPrinter ip, boolean skipPackageImport) {
        printPackageInfo(inst.getComponentType().getReferencedSymbol(), ip, skipPackageImport);
        ip.print("component /*instance*/ " + inst.getName());

        ip.println(" {");
        printTags(inst, ip);

        ip.indent();

        printPorts(inst.getPortInstanceList(), ip);

        inst.getSubComponents().stream().forEachOrdered(a -> printEMAComponentInstance(a, ip, true));

        inst.getConnectorInstances().stream().forEachOrdered(a -> {
            ip.print("connect ");
            printConnector(a, ip);
            ip.println(";");
        });

        ip.unindent();
        ip.println("}");
    }

    public static String printEMAComponentInstance(EMAComponentInstanceSymbol inst) {
        IndentPrinter ip = new IndentPrinter();
        printEMAComponentInstance(inst, ip, false);
        return ip.getContent();
    }

    public static String printEMAComponentInstanceAsEMAComponent(EMAComponentInstanceSymbol instance) {
        IndentPrinter ip = new IndentPrinter();
        printEMAComponentInstanceAsEMAComponent(instance, ip);
        return ip.getContent();
    }


    public static void printEMAComponentInstanceAsEMAComponent(EMAComponentInstanceSymbol instance, IndentPrinter ip) {

        ip.print("component " + capitalize(normalize(instance.getName())));
        ip.println("{");
        ip.indent();

        instance.getPortInstanceList()
                .stream()
                .map(p -> String.format("port %s %s %s;",
                        p.isIncoming() ? "in" : "out",
                        p.getTypeReference().getName(),
                        normalize(p.getName())))
                .forEach(ip::println);

        ip.println();

        instance.getSubComponents()
                .forEach(inst -> printEMAComponentInstanceAsEMAComponent(inst, ip));

        ip.println();

        instance.getSubComponents()
                .stream()
                .map(inst -> "instance " + capitalize(normalize(inst.getName())) + " " + normalize(inst.getName()) + ";")
                .forEach(ip::println);

        ip.println();

        instance.getConnectorInstances()
                .stream()
                .map(con -> String.format("connect %s -> %s;",
                        normalize(con.getSource()),
                        normalize(con.getTarget())))
                .forEach(ip::println);

        ip.unindent();
        ip.println("}");
    }

    private static String normalize(String s){
        return s.replace("[", "_").replace("]","");
    }

    private static String capitalize(String s){
        return s.substring(0,1).toUpperCase() + s.substring(1);
    }
}
