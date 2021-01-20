/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util;

import com.google.common.base.Joiner;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.executionOrder.Call;
import de.monticore.lang.monticar.semantics.executionOrder.SList;
import de.monticore.lang.monticar.semantics.executionOrder.SListEntry;
import de.monticore.lang.monticar.semantics.helper.Find;
import de.monticore.prettyprint.IndentPrinter;

import java.util.List;

import static de.monticore.lang.monticar.semantics.helper.EMAPropertiesHelper.isAtomic;
import static de.monticore.lang.monticar.semantics.helper.EMAPropertiesHelper.isNonVirtual;

public class PrintExecutionOrder {

    public static String printExecutionOrder(EMAComponentInstanceSymbol rootComponent) {
        IndentPrinter printer = new IndentPrinter();

        for (EMAComponentInstanceSymbol component : Find.allAtomicOrNVComponents(rootComponent)) {
            printer.println(getPartialName(component, rootComponent));
            printer.indent();
            printer.println(String.format("Output: %s", Joiner.on(',').join(component.getOrderOutput())));
            printer.println(String.format("Update: %s", component.getOrderUpdate()));
            if (isNonVirtual(component) && !isAtomic(component))
                printer.println(printExecutionOrder(component));
            printer.unindent();
        }

        return printer.getContent();
    }

    public static String printSListSerial(EMAComponentInstanceSymbol rootComponent) {
        return printSList(SList.sListSerial(rootComponent), rootComponent);
    }

    public static String printSList(List<SListEntry> sList) {
        IndentPrinter printer = new IndentPrinter();

        int i = 1;
        for (SListEntry entry : sList) {
            printer.println(String.format("%s) %s: %s", i++,
                    entry.getComponent().getFullName(),
                    printCall(entry.getCall())));
            printer.unindent();
        }

        return printer.getContent();
    }

    public static String printSList(List<SListEntry> sList, EMAComponentInstanceSymbol relativeComponent) {
        IndentPrinter printer = new IndentPrinter();

        int i = 1;
        for (SListEntry entry : sList) {
            printer.println(String.format("%s) %s: %s", i++,
                    getPartialName(entry.getComponent(), relativeComponent),
                    printCall(entry.getCall())));
            printer.unindent();
        }

        return printer.getContent();
    }

    private static String printCall(Call call) {
        switch (call) {
            case OUTPUT: return "output";
            case UPDATE: return "update";
            case EXECUTE: return "execute";
            default: return "execute";
        }
    }

    private static String getPartialName(EMAComponentInstanceSymbol component,
        EMAComponentInstanceSymbol rootComponent) {
        String fullName = component.getFullName();
        String rFullName = rootComponent.getFullName();

        if (fullName.length() < rFullName.length()) return fullName;
        if (!fullName.startsWith(rFullName)) return fullName;
        if (fullName.equals(rFullName)) return fullName;
        return fullName.substring(rFullName.length() + 1);
    }
}
