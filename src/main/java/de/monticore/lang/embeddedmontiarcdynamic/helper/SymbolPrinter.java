/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicEventHandlerSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicPortArraySymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.*;
import de.monticore.lang.monticar.helper.IndentPrinter;

import java.util.Collection;
import java.util.Iterator;
import java.util.stream.Collectors;

public class SymbolPrinter extends de.monticore.lang.embeddedmontiarc.helper.SymbolPrinter {

    public static String printDynamicComponent(EMADynamicComponentSymbol cmp) {
        IndentPrinter ip = new IndentPrinter();
        printDynamicComponent(cmp, ip, false);
        return ip.getContent();
    }

    public static void printDynamicComponent(EMADynamicComponentSymbol cmp, IndentPrinter ip, boolean skipPackageImport) {
        printPackageInfo(cmp, ip, skipPackageImport);
        if(cmp.isDynamic()){
            ip.print("dynamic ");
        }
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

        printDynamicPorts(cmp.getPortsList(), ip);

        cmp.getDynamicSubComponents().stream().forEachOrdered(a -> {
            ip.print("instance ");
            printEMADynamicComponentInstantiation(a, ip);
            ip.println(";");
        });

        cmp.getInnerDynamicComponents().stream().forEachOrdered(a -> printDynamicComponent(a, ip, true));

        cmp.getConnectors().stream().forEachOrdered(a -> {
            ip.print("connect ");
            printConnector(a, ip);
            ip.println(";");
        });

        //print event handler
        cmp.getEventHandlers().stream().forEachOrdered(e -> {
            printEventHandler(e, ip);
        });

        ip.unindent();
        ip.println("}");
    }

    public static void printDynamicPorts(Collection<? extends EMAPortSymbol> ports, IndentPrinter ip) {
        if (!ports.isEmpty()) {
            ip.println("ports");
            ip.indent();
            int i = 0;
            int s = ports.size();
            for (EMAPortSymbol p : ports) {
//                if(p instanceof EMADynamicPortSymbol){
//                    printPort((EMADynamicPortSymbol) p, ip);
//            }else if(p instanceof EMADynamicPortArraySymbol){
                if(p instanceof EMADynamicPortArraySymbol){
//                    EMADynamicPortArraySymbol dp = (EMADynamicPortArraySymbol)p;
                    printDynamicPortArray((EMADynamicPortArraySymbol)p, ip);
                    //printPort((EMADynamicPortSymbol) p, ip);
//                    ip.println("TODO: p instanceof EMADynamicPortArraySymbol");
                }else{
                    printPort(p, ip);
                }

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

    public static void printDynamicPortArray(EMADynamicPortArraySymbol port, IndentPrinter ip) {
        if(port.isDynamic()){
            ip.print("dynamic ");
        }

        if (port.isIncoming()) {
            ip.print("in ");
        } else {
            ip.print("out ");
        }

        ip.print(port.getTypeReference().getName());
        ip.print(printTypeParameters(port.getTypeReference().getActualTypeArguments()));
        ip.print(" ");
        ip.print(port.getName());

        ip.print("[");
        ip.print(port.getNameNonDynamicSizeDependsOn().orElse(Integer.toString(port.getNonDynamicDimension())));
        ip.print(":");
        if(port.isDimensionInfinite()){
            ip.print("oo");
        }else {
            ip.print(port.getNameDependsOn().orElse(Integer.toString(port.getDimension())));
        }
        ip.print("]");

        printTags(port, ip);
    }

    public static void printEventHandler(EMADynamicEventHandlerSymbol eve, IndentPrinter ip){
        ip.print("@ ");
        ip.print(eve.getCondition().getTextualRepresentation());
        ip.println(" {");
        ip.indent();

        eve.getDynamicSubComponents().stream().forEachOrdered(a -> {
            ip.print("instance ");
            printEMADynamicComponentInstantiation(a, ip);
            ip.println(";");
        });

        //print connectors
        eve.getConnectors().stream().forEachOrdered(a -> {
            ip.print("connect ");
            printConnector(a, ip);
            ip.println(";");
        });

        //print sub eventhandlers
        eve.getEventHandlers().stream().forEachOrdered(e -> {
            printEventHandler(e, ip);
        });


        ip.unindent();
        ip.println("}");
    }

    public static void printEMADynamicComponentInstantiation(EMADynamicComponentInstantiationSymbol inst, IndentPrinter ip){
        ip.print(inst.getComponentType().getName());
        ip.print(printTypeParameters(inst.getComponentType().getActualTypeArguments()));
        ip.print(printConfigArguments(inst.getConfigArguments()));

        //dynamic
        if(inst.isDynamic()) {
            ip.print(" /* dynamic */");
        }
        ip.print(" ");
        ip.print(inst.getName());

        if(inst.isArray()){
            ip.print(" [");

            if(inst.isDynamic()){
                ip.print(inst.getNonDynamicDimensionDependsOn().orElse(Integer.toString(inst.getNonDynamicDimension())));
                ip.print(":");
                if(inst.isDimensionInfinite()){
                    ip.print("oo");
                }else {
                    ip.print(inst.getDimensionDependsOn().orElse(Integer.toString(inst.getDimension())));
                }
            }else {
                ip.print(inst.getDimensionDependsOn().orElse(Integer.toString(inst.getDimension())));
            }

            ip.print("]");
        }
    }




    public static void printDynamicComponentInstance(EMADynamicComponentInstanceSymbol inst, IndentPrinter ip, boolean skipPackageImport) {
        printPackageInfo(inst.getComponentType().getReferencedSymbol(), ip, skipPackageImport);

        if(inst.isDynamic()){
            ip.print("dynamic ");
        }
        ip.print("component ");

        if(inst.isDynamicInstance()){
            ip.print("/* DYNAMIC instance*/ dynamic ");
        }else{
            ip.print("/*instance*/ ");
        }
        ip.print(inst.getName());
//        if(inst.isDynamicInstance()){
//            ip.print(" [");
//            ip.print(inst.getNumberOfNonDynamicInstances());
//            ip.print(":");
//            ip.print(inst.getNumberOfDynamicInstances());
//            ip.print("]");
//        }else{
//            if(inst.isArray()){
//                ip.print(" [");
//                ip.print(inst.getNumberOfDynamicInstances());
//                ip.print("]");
//            }
//        }

        ip.println(" {");
        printTags(inst, ip);
        ip.indent();
//        printDynamicPortInstanceSymbols(inst.getDynamicPortInstanceSymbolList(), ip);

        printPortsWithDynamic(inst.getPortInstanceList(), ip);

        inst.getDynamicSubComponents().stream().forEachOrdered((a) -> {
            printDynamicComponentInstance(a, ip, true);
        });
        inst.getConnectorInstances().stream().forEachOrdered((a) -> {
            ip.print("connect ");
            printConnector(a, ip);
            ip.println(";");
        });

        inst.getEventHandlers().stream().forEach(eh -> {
            printDynamicEventHandlerInstance(eh, ip);
        });

        ip.unindent();
        ip.println("}");
    }

    public static String printDynamicComponentInstance(EMADynamicComponentInstanceSymbol inst) {
        IndentPrinter ip = new IndentPrinter();
        printDynamicComponentInstance(inst, ip, false);
        return ip.getContent();
    }

    public static  void printDynamicEventHandlerInstance(EMADynamicEventHandlerInstanceSymbol inst, IndentPrinter ip){
        ip.print("@ /*instance */ ");
        ip.print(inst.getCondition().getTextualRepresentation());
        ip.println("{");
        ip.indent();
        inst.getConnectors().stream().forEachOrdered(a -> {
            ip.print("connect ");
            printConnector(a, ip);
            ip.println(";");
        });
        ip.unindent();
        ip.println("}");

    }

    public static void printPortsWithDynamic(Collection<EMAPortInstanceSymbol> ports, IndentPrinter ip){
        if (!ports.isEmpty()) {
            ip.println("ports");
            ip.indent();
            int i = 0;

            for(EMAPortInstanceSymbol port : ports){
                if(port instanceof EMADynamicPortInstanceSymbol){
                    printEMADynamicPortInstanceSymbol((EMADynamicPortInstanceSymbol) port, ip);
                }else {
                    printPort(port, ip);
                }
                if (i == ports.size() - 1) {
                    ip.println(";");
                } else {
                    ip.println(",");
                }
                ++i;
            }

            ip.unindent();
        }
    }

    public static void printEMADynamicPortInstanceSymbol(EMADynamicPortInstanceSymbol port, IndentPrinter ip ){
        if(port.isDynamic()){
            ip.print("dynamic ");
        }
        if(port.isIncoming()){
            ip.print("in ");
        }else{
            ip.print("out ");
        }

        ip.print(port.getTypeReference().getName());
        ip.print(printTypeParameters(port.getTypeReference().getActualTypeArguments()));
        ip.print(" ");
        ip.print(port.getName());

        printTags(port, ip);
    }


}
