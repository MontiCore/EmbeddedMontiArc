package de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.templatebuilder.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.enums.Enums;
import freemarker.ext.util.WrapperTemplateModel;
import freemarker.template.*;

import java.util.List;

public class EventBodyClashDetector implements TemplateMethodModelEx {

    /*
     * Creates two or three list of connectors. Adds error messages to the output if two connectors access the same target port (overlap)
     * Three comparisons possible:
     *  1. List in reconfiguration compares for an overlap in the list itself
     *  2. List in reconfiguration compared to list of initial connectors (list of connectors outside of any reconfiguration) for overlaps
     *  3. Lists in two reconfigurations are compared with each other for overlaps
     */

    public Object exec(List list) throws TemplateModelException {
        StringBuilder messageBuilder = new StringBuilder();

        List<EMAConnectorSymbol> initialTargets = (List<EMAConnectorSymbol>) ((WrapperTemplateModel) list.get(0)).getWrappedObject();

        if (list.size() == 1)
            checkForSelfDuplicate(initialTargets, messageBuilder);
        else if (list.size() >= 2) {
            List<EMAConnectorSymbol> eventBodysOfEvent1 = (((List<EMAConnectorSymbol>) ((WrapperTemplateModel) list.get(1)).getWrappedObject()));
            if (list.size() == 2) {
                checkForSelfDuplicate(eventBodysOfEvent1, messageBuilder);
                comparePairWise(initialTargets, eventBodysOfEvent1, messageBuilder);
            } else if (list.size() >= 3) {
                List<EMAConnectorSymbol> eventBodysOfEvent2 = (((List<EMAConnectorSymbol>) ((WrapperTemplateModel) list.get(2)).getWrappedObject()));
                comparePairWise(eventBodysOfEvent1, eventBodysOfEvent2, messageBuilder);
            }
        }
        if (messageBuilder.length() == 0)
            return Enums.LogMessageTypes.CONFLICTFREE.toString() + ":";

        return Enums.LogMessageTypes.PORTOVERLAP + ":" + messageBuilder.toString(); //pattern: PORTOVERLAP:{portName,lineNr1,lineNr2};
    }

    /**
     * checks if the ports in the list @source contains a port name twice or more, and if so, adds a message to the StringBuilder
     */
    private static void checkForSelfDuplicate(List<EMAConnectorSymbol> source, StringBuilder builder) {
        for (int i = 0; i < source.size(); i++) {
            for (int j = i + 1; j < source.size(); j++) {
                if (source.get(i).getTarget().equals(source.get(j).getTarget()))
                    builder.append("" +
                            String.format("{%1$s,%2$s,%3$s};", source.get(i).getName(), source.get(i).getSourcePosition().getLine(), source.get(j).getSourcePosition().getLine()));
            }
        }
    }

    /**
     * compares if two lists contain any port name twice or more, and if so, adds a message to the StringBuilder
     **/
    private static void comparePairWise(List<EMAConnectorSymbol> source1, List<EMAConnectorSymbol> source2, StringBuilder builder) {
        for (int i = 0; i < source1.size(); i++) {
            for (int j = 0; j < source2.size(); j++) {
                if (!source1.get(i).getTarget().endsWith("[?]") && !source2.get(j).getTarget().endsWith("[?]")) //allowed, because scope changed
                    if (source1.get(i).getTarget().equals(source2.get(j).getTarget()))
                        builder.append("" +
                                String.format("{%1$s,%2$s,%3$s};", source1.get(i).getName(), source1.get(i).getSourcePosition().getLine(), source2.get(j).getSourcePosition().getLine()));
            }
        }
    }
}