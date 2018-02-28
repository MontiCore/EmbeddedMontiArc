/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.emadl.generator;


import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.apache.commons.cli.*;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

public class GeneratorCli{

    public static final Option OPTION_MODELS_PATH = Option.builder("m")
            .longOpt("models-dir")
            .desc("full path to directory with EMAM models e.g. C:\\Users\\vpupkin\\proj\\MyAwesomeAutopilot\\src\\main\\emam")
            .hasArg(true)
            .required(true)
            .build();

    public static final Option OPTION_ROOT_MODEL = Option.builder("r")
            .longOpt("root-model")
            .desc("fully qualified name of the root model e.g. de.rwth.vpupkin.modeling.mySuperAwesomeAutopilotComponent")
            .hasArg(true)
            .required(true)
            .build();

    public static final Option OPTION_OUTPUT_PATH = Option.builder("o")
            .longOpt("output-dir")
            .desc("full path to output directory for tests e.g. C:\\Users\\vpupkin\\proj\\MyAwesomeAutopilot\\target\\gen-cpp")
            .hasArg(true)
            .required(false)
            .build();

    private GeneratorCli() {
    }

    public static void main(String[] args) {
        Options options = getOptions();
        CommandLineParser parser = new DefaultParser();
        CommandLine cliArgs = parseArgs(options, parser, args);
        if (cliArgs != null) {
            runGenerator(cliArgs);
        }
    }

    private static Options getOptions() {
        Options options = new Options();
        options.addOption(OPTION_MODELS_PATH);
        options.addOption(OPTION_ROOT_MODEL);
        options.addOption(OPTION_OUTPUT_PATH);
        return options;
    }

    private static CommandLine parseArgs(Options options, CommandLineParser parser, String[] args) {
        CommandLine cliArgs;
        try {
            cliArgs = parser.parse(options, args);
        } catch (ParseException e) {
            System.err.println("argument parsing exception: " + e.getMessage());
            System.exit(1);
            return null;
        }
        return cliArgs;
    }

    private static void runGenerator(CommandLine cliArgs) {
        Path modelsDirPath = Paths.get(cliArgs.getOptionValue(OPTION_MODELS_PATH.getOpt()));
        String rootModelName = cliArgs.getOptionValue(OPTION_ROOT_MODEL.getOpt());
        String outputPath = cliArgs.getOptionValue(OPTION_OUTPUT_PATH.getOpt());
        Generator generator = new Generator();
        if (outputPath != null){
            generator.setGenerationTargetPath(outputPath);
        }
        try{
            generator.generate(modelsDirPath, rootModelName);
        }
        catch (IOException e){
            Log.error("io error during generation", e);
            System.exit(1);
        }
        catch (TemplateException e){
            Log.error("template error during generation", e);
            System.exit(1);
        }
    }
}
