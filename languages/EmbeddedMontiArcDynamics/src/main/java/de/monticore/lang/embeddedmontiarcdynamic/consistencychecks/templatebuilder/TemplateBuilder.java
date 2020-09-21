package de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.templatebuilder;

import de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.templatebuilder.helper.EventBodyClashDetector;
import de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.templatebuilder.helper.InstanceOfHelper;
import de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.templatebuilder.helper.PortSequenceHelper;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicPortArraySymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.*;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues.PortValueCompareSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues.PortValueInputSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues.PortValuesArraySymbol;
import de.se_rwth.commons.logging.Log;
import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import freemarker.template.TemplateExceptionHandler;

import java.io.*;
import java.util.*;

import static de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.enums.Enums.LogMessageTypes.EQUALCONDITIONS;
import static de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.enums.Enums.LogMessageTypes.*;

public class TemplateBuilder {

    private static final Template _TEMPLATE;
    private static final String Z3TEMPFIleName = ".consistencycheck.z3.tmp";    //if changed, do also change in .gitignore

    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(TemplateBuilder.class, "/freemarker");

        try {
            _TEMPLATE = conf.getTemplate("consistencychecktemplate.ftl");
            _TEMPLATE.setLocale(Locale.ENGLISH);            //decimal separator is a dot
            _TEMPLATE.setNumberFormat("#.################");//no thousands separator
        } catch (IOException e) {
            String msg = "could not load templates";
            Log.error(msg, e);
            throw new RuntimeException(msg, e);
        }
    }

    public static String generateTemplate(EMADynamicComponentSymbol c) {
        Map<String, Object> data = getDataForTemplate(c);

        Log.errorIfNull(_TEMPLATE);
        Log.errorIfNull(data);

        StringWriter sw = new StringWriter();
        try {
            _TEMPLATE.process(data, sw);
        } catch (TemplateException | IOException e) {
            Log.error("template generation failed, template: " + _TEMPLATE.getName(), e);
        }
        return sw.toString();
    }

    public static Map<String, Object> getDataForTemplate(EMADynamicComponentSymbol symTab) {
        HashMap<String, Object> data = new HashMap<>();

        //the symbol table
        data.put("symTab", Log.errorIfNull(symTab));

        //helper classes
        data.put("instanceOf", new InstanceOfHelper());
        data.put("clashCheck", new EventBodyClashDetector());
        data.put("PortInSequenceAlreadyDefined", new PortSequenceHelper());

        //list of ports that occur in sequences and were already added with freemarker
        data.put("DefinedPortsInSequences", new ArrayList<String>());

        data.put("PREAMBLE", PREAMBLE.toString());
        data.put("INITIALLYSATISFIABLE", INITIALLYSATISFIABLE.toString());
        data.put("SINGLECONDNOTSAT", SINGLECONDITIONNOTSATISFIABLE.toString());
        data.put("EQUALCONDITIONS", EQUALCONDITIONS.toString());
        data.put("INVALIDBOUNDARY", INVALIDBOUNDARY.toString());

        //lost of connectors 'a -> b' outside of reconfigurations
        data.put("initialConnects", symTab.getConnectors());

        //class instances needed for using the 'instanceof' helper
        data.put("EventBracketExpressionSymbol", EventBracketExpressionSymbol.class);
        data.put("EventLogicalOperationExpressionSymbol", EventLogicalOperationExpressionSymbol.class);
        data.put("EventPortExpressionValueSymbol", EventPortExpressionValueSymbol.class);
        data.put("EventPortExpressionConnectSymbol", EventPortExpressionConnectSymbol.class);
        data.put("EventBooleanExpressionSymbol", EventBooleanExpressionSymbol.class);
        data.put("EventPortExpressionFreeSymbol", EventPortExpressionFreeSymbol.class);
        data.put("PortValueInputSymbol", PortValueInputSymbol.class);
        data.put("PortValueCompareSymbol", PortValueCompareSymbol.class);
        data.put("PortValuesArraySymbol", PortValuesArraySymbol.class);
        data.put("EMADynamicPortArraySymbol", EMADynamicPortArraySymbol.class);

        return data;
    }

    public BufferedReader performCheck(String txt) {
        try (PrintWriter out = new PrintWriter(Z3TEMPFIleName)) {
            out.println(txt);
        } catch (FileNotFoundException e) {
            String msg = String.format("could not print file '%s'", Z3TEMPFIleName);
            Log.error(msg, e);
            e.printStackTrace();
        }

        try {
            String commandString;
            if (System.getProperty("os.name").toLowerCase().contains("windows")) {
                //windows: call cmd first
                commandString = "cmd.exe /C z3 " + Z3TEMPFIleName;
            } else {
                commandString = "z3 " + Z3TEMPFIleName;
            }
            Date durationZ3 = new Date();
            Process process = Runtime.getRuntime().exec(commandString);
            BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
            return reader;
        } catch (Exception e) {
            String msg = "could not execute the z3 solver";
            Log.error(msg, e);
            e.printStackTrace();
        }
        return null;
    }
}
