package de.monticore.lang.embeddedmontiarcdynamic.consistencychecks;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.outputparser.OutputParser;
import de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.templatebuilder.TemplateBuilder;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Date;

import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbol;
import de.se_rwth.commons.logging.Log;

public class ConsistencyChecker {

    public ConsistencyChecker() {

    }

    /**
     * @param symbolTable the symbol table object of the component that should be analyzed
     */
    public void Check(EMADynamicComponentSymbol symbolTable) {
        if (isZ3Installed()) {
            TemplateBuilder templateBuilder = new TemplateBuilder();
            String output = templateBuilder.generateTemplate(symbolTable);
            BufferedReader br = templateBuilder.performCheck(output);

            try {
                new OutputParser().parse(br);
            } catch (IOException e) {
                String msg = "could not parse output";
                Log.error(msg, e);
            }
            return;
        } else {
            Log.warn("z3 program not installed. Skipping testing of consistency!");
        }
    }


    public void CheckWithPerformance(EMADynamicComponentSymbol symbolTable) {
        if (isZ3Installed()) {
            TemplateBuilder templateBuilder = new TemplateBuilder();
            Date generateTemplateStart = new Date();
            String output = templateBuilder.generateTemplate(symbolTable);
            Log.info(String.format("Generation of z3 code:      %1$s seconds", (1.0 * (new Date().getTime() - generateTemplateStart.getTime()) / 1000)), "Performance");
            Date durationz3Analysis = new Date();
            BufferedReader br = templateBuilder.performCheck(output);

            try {
                new OutputParser().parse(br);
                Log.info(String.format("Execution of z3:            %1$s seconds", (1.0 * (new Date().getTime() - durationz3Analysis.getTime())) / 1000), "Performance");
            } catch (Exception e) {
                String msg = "could not parse output";
                Log.error(msg, e);
            }
            return;
        } else {
            Log.warn("z3 program not installed. Skipping testing of consistency!");
        }
    }

    /*
     * returns true, if z3 is installed. Therefore, it calls 'z3 -version' that should correctly output e.g. 'Z3 version 4.8.5 - 64 bit'
     */
    private boolean isZ3Installed() {
        String commandString;
        if (System.getProperty("os.name").toLowerCase().contains("windows")) {
            commandString = "cmd.exe /C z3 -version";
        } else {
            commandString = "z3 -version";
        }

        try {
            Process proc = Runtime.getRuntime().exec(commandString);
            BufferedReader reader = new BufferedReader(new InputStreamReader(proc.getInputStream()));
            String version = reader.readLine();
            return version.toLowerCase().startsWith("z3 version");
        } catch (Exception e) {
            Log.error("could not retrieve z3 version", e);
            e.printStackTrace();
            return false;
        }
    }
}
