/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.tools;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.order.ImplementExecutionOrder;
import de.monticore.lang.monticar.generator.order.NonVirtualBlock;
import de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema.TagExecutionOrderSymbol;
import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.prettyprint.IndentPrinter;
import de.se_rwth.commons.logging.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.Collection;
import java.util.List;


public class Slist extends AbstractSymtab {

    private static int nonVirtualBlockSize = 0;

    // for executing through command line
    public static void main(String[] args) throws IOException {
        if (!(new File(args[0] + ".arc").exists())) {
            IndentPrinter ip = new IndentPrinter();
            ip.println("File does not exist!");
            ip.println();
            ip.println("Syntax: slist componentName");
            ip.println("Examples:");
            ip.indent();
            ip.println("slist Distronic");
            ip.println("slist BrakeForceAssistant");
            ip.unindent();
            Log.debug(ip.getContent(), "Slist.main");
//            System.out.println(ip.getContent());
        } else {
            //creates a fileReader for the *.arc file
            FileReader fr = new FileReader(args[0] + ".arc");
            BufferedReader br = new BufferedReader(fr);

            //If the first line starts with 'package ' then the package will be stored
            // otherwise the component name
            String packageArc = br.readLine();
            String componentName;
            if (packageArc.substring(0, 8).equals("package ")) {
                packageArc = packageArc.substring(8, packageArc.length() - 1);
                componentName = packageArc + "." + args[0].substring(args[0].lastIndexOf("\\") + 1, args[0].length());
            } else {
                componentName = args[0].substring(args[0].lastIndexOf("\\") + 1, args[0].length());
            }
            // saves the modelPath as the given Path without the package
            String modelPath = args[0].replace(componentName.replace(".", "\\"), "");
            modelPath = modelPath.substring(0, modelPath.length() - 1);
            br.close();

            TaggingResolver symTab = createSymTabAndTaggingResolver(modelPath);
            EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                    NameHelper.toInstanceFullQualifiedName(componentName),
                    EMAComponentInstanceSymbol.KIND).orElse(null);
            Log.debug(execute(symTab, inst), "Slist.main");
//            System.out.println(execute(symTab, inst));
        }
    }

    // creates an indent printer and prints the first line of slist.
    // After this printComponentBlockOrder will be executed to print all components
    public static String execute(TaggingResolver taggingResolver, EMAComponentInstanceSymbol inst) {
        IndentPrinter ip = new IndentPrinter();
        List<EMAComponentInstanceSymbol> exOrder = ImplementExecutionOrder.exOrder(taggingResolver, inst);
        getNonVirtualBlockSize(taggingResolver, inst);

        ip.println("---- Sorted list for '" + inst.getName() + "' [" + nonVirtualBlockSize +
                " nonvirtual block(s), directfeed=" + getDirectFeedSize(inst) + "]");
        printComponentBlockOrder(taggingResolver, exOrder, ip);
        return ip.getContent();
    }

    // This method prints every component
    public static void printComponentBlockOrder(TaggingResolver taggingResolver, List<EMAComponentInstanceSymbol> exOrder, IndentPrinter ip) {
        for (EMAComponentInstanceSymbol order : exOrder) {
            ip.indent();
            ip.print(((TagExecutionOrderSymbol) taggingResolver.getTags(order, TagExecutionOrderSymbol.KIND)
                    .iterator().next()).getExecutionOrder());
            ip.print("    ");
            ip.print("'" + order.getFullName() + "' (");
            ip.print(order.getComponentType().getName() + ", ");
            if (order.getIncomingPortInstances().isEmpty() && !order.getActualTypeArguments().isEmpty()) {
                ip.println("tid=PRM)");
            } else {
                ip.println("tid=0)");
            }
            ip.unindent();
        }
    }

    // Counts how much NonVirtualBlocks are in the component
    public static int getNonVirtualBlockSize(TaggingResolver taggingResolver, EMAComponentInstanceSymbol inst) {
        for (EMAComponentInstanceSymbol subInst : inst.getSubComponents()) {
            if (taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).isEmpty()
                    && !subInst.getSubComponents().isEmpty()) {
                getNonVirtualBlockSize(taggingResolver, subInst);
            }
            if (!taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).isEmpty()
                    && taggingResolver.getTags(subInst, TagExecutionOrderSymbol.KIND).size() == 1) {
                nonVirtualBlockSize++;
            }
        }
        return nonVirtualBlockSize;
    }

    // Counts how much DirectFeeds in the component are present
    public static int getDirectFeedSize(EMAComponentInstanceSymbol inst) {
        //TODO Implementation
        return 0;
    }

    public static void resetNonVirtualBlockSize() {
        nonVirtualBlockSize = 0;
    }


}
