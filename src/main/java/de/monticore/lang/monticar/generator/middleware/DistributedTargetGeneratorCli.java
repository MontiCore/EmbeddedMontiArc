package de.monticore.lang.monticar.generator.middleware;

import com.google.gson.Gson;
import com.google.gson.stream.JsonReader;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.monticar.emadl.generator.EMADLAbstractSymtab;
import de.monticore.lang.monticar.emadl.generator.EMADLGeneratorCli;
import de.monticore.lang.monticar.generator.middleware.impls.CPPGenImpl;
import de.monticore.lang.monticar.generator.middleware.impls.EMADLGeneratorImpl;
import de.monticore.lang.monticar.generator.middleware.impls.ODVGenImpl;
import de.monticore.lang.monticar.generator.middleware.impls.RosCppGenImpl;
import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.cli.*;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

public final class DistributedTargetGeneratorCli {

    private static final String helpString = "Parameters: <file path to config json> OR -r <raw json config string>\n\n" +
            "Schema of config json:\n" +
            "{\n" +
            "    'modelsDir':'<path to directory with EMAM models>',\n" +
            "    'outputDir':'<path to output directory for generated files>',\n" +
            "    'rootModel':'<fully qualified name of the root model>',\n" +
            "    'generators':['<identifier for first generator>', '<identifier for second generator>',...],\n" +
            "    'emadlBackend':'<deep-learning-framework backend. Options: MXNET, CAFFE2>'\n" +
            "}\n\n" +
            "Generator Options:\n" +
            "Behaviour generators:\n" +
            "    'cpp': EMAM2CPP\n" +
            "    'emadlcpp': EMADL2CPP\n" +
            "Middleware generators:\n" +
            "    'roscpp': EMAM2Roscpp";

    public static final String GENERATOR_CPP = "cpp";
    public static final String GENERATOR_EMADL = "emadlcpp";
    public static final String GENERATOR_ROSCPP = "roscpp";
    public static final String GENERATOR_ODV = "odv";

    private DistributedTargetGeneratorCli() {}

    public static void main(String[] args) {
        System.out.println(Arrays.toString(args));
        Gson gson = new Gson();
        CliParameters parameters = null;
        //File name only
        if(args.length == 1){

            String filePath = expandHomeDir(args[0]);
            try {
                JsonReader jsonReader = new JsonReader(new FileReader(filePath));
                parameters = gson.fromJson(jsonReader, CliParameters.class);
            } catch (FileNotFoundException e) {
                Log.error("0x49E6A: Can not find specified config file: " + args[0] + "!");
            }

        }
        //raw mode
        else if(args.length == 2 && args[0].equals("-r")){
            parameters = gson.fromJson(args[1], CliParameters.class);
        }else{
            System.out.println(helpString);
        }

        if(parameters != null){
            runGenerator(parameters);
        }
    }

    private static Set<String> getGeneratorNames() {
        HashSet<String> res = new HashSet<>();
        res.add(GENERATOR_CPP);
        res.add(GENERATOR_EMADL);
        res.add(GENERATOR_ROSCPP);
        res.add(GENERATOR_ODV);
        return res;
    }

    public static void runGenerator(CliParameters cliParameters) {
        if(cliParameters.getModelsDir() == null){
            Log.error("0xEB6FE: No models dir specified!");
            return;
        }
        String fullModelsDirPath = expandHomeDir(cliParameters.getModelsDir());

        if(cliParameters.getGenerators().size() == 0){
            Log.error("0x6178E: No generator was specified!");
            return;
        }

        if(!Files.isDirectory(Paths.get(fullModelsDirPath))){
            Log.error("0x6444B: Models dir does not exist: " + fullModelsDirPath);
            return;
        }

        TaggingResolver taggingResolver;
        if (cliParameters.getGenerators().contains(GENERATOR_EMADL)) {
            taggingResolver = EMADLAbstractSymtab.createSymTabAndTaggingResolver(fullModelsDirPath);
        }
        else{
            taggingResolver = AbstractSymtab.createSymTabAndTaggingResolver(fullModelsDirPath);
        }

        DistributedTargetGenerator generator = new DistributedTargetGenerator();
        generator.setGenerationTargetPath(cliParameters.getOutputDir());

        Set<String> validGenNames = getGeneratorNames();

        cliParameters.getGenerators().forEach(genName -> {
            if (validGenNames.contains(genName)) {
                Log.warn("Using generator " + genName);
            } else {
                Log.error("0xE28B6: Not a valid generator Name:" + genName  +".");
                return;
            }
        });

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve(cliParameters.getRootModel(), ExpandedComponentInstanceSymbol.KIND).orElse(null);

        if (componentInstanceSymbol == null) {
            Log.error("0x5FFAE: The given component cannot be resolved.");
            return;
        }

        if (cliParameters.getGenerators().contains(GENERATOR_CPP)) {
            generator.add(new CPPGenImpl(), "cpp");
        }

        if (cliParameters.getGenerators().contains(GENERATOR_EMADL)) {
            if(cliParameters.getEmadlBackend() != null && !cliParameters.getEmadlBackend().equals("")) {
                generator.add(new EMADLGeneratorImpl(fullModelsDirPath, cliParameters.getEmadlBackend()), "cpp");
            }else{
                Log.error("0x0D839: Generator EMADL was specified but no backend was selected!");
                return;
            }
        }

        if (cliParameters.getGenerators().contains(GENERATOR_ROSCPP)) {
            generator.add(new RosCppGenImpl(), "roscpp");
            RosToEmamTagSchema.registerTagTypes(taggingResolver);
            TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);
        }

        if (cliParameters.getGenerators().contains(GENERATOR_ODV)) {
            generator.add(new ODVGenImpl(), "odv");
        }

        try {
            generator.generate(componentInstanceSymbol, taggingResolver);
        } catch (IOException e) {
            Log.error("Error generating files", e);
        }
    }

    private static String expandHomeDir(String path) {
        if (path.startsWith("~")) {
            return System.getProperty("user.home") + path.substring(1);
        }
        return path;
    }
}
