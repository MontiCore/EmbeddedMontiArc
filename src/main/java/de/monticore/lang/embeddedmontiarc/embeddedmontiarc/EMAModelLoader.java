package de.monticore.lang.embeddedmontiarc.embeddedmontiarc;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.InstancingRegister;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.NoSuchElementException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;


public class EMAModelLoader {


    public static EMAComponentSymbol loadComponentFromMainTxt(String mainTxt) {
        Scope scope = createSymTabFromMainTxt(mainTxt);
        String mainComponent = parseMainComponent(mainTxt);

        EMAComponentSymbol emaComponentSymbol = scope.<EMAComponentSymbol>resolve(mainComponent, EMAComponentSymbol.KIND).orElse(null);

        if(emaComponentSymbol == null)
            Log.error("Could not resolve mainComponent form mainTxt: " + mainComponent);

        return emaComponentSymbol;
    }

    public static Scope createSymTabFromMainTxt(String mainTxt) {
        String mainComponent = parseMainComponent(mainTxt);
        String mainInstantiation = parseMainInstantiation(mainTxt);
        ModelPath modelPath = parseModelPath(mainTxt);

        Scope scope = createSymTab(modelPath, mainComponent, mainInstantiation);
        return scope;
    }

    protected static Scope createSymTab(ModelPath modelPath, String mainComponent, String mainInstantiation) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        fam.addModelingLanguage(new EmbeddedMontiArcLanguage(mainComponent, mainInstantiation));
        fam.addModelingLanguage(new StreamLanguage());
        fam.addModelingLanguage(new StructLanguage());

        GlobalScope scope = new GlobalScope(modelPath, fam);

        de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);

        LogConfig.init();
        InstancingRegister.reset();
        return scope;
    }

    private static String parseMainComponent(String mainTxt) {
        String mainComponent = "defaultComponent";
        try {
            Pattern p = Pattern.compile("Main-Component-Instantiation:\\s+(\\w+(?:\\.\\w+)*)\\s+(\\w+);");
            Stream<String> lines = Files.lines(Paths.get(mainTxt));
            mainComponent = lines.map(p::matcher)
                    .filter(Matcher::matches)
                    .findFirst().get().group(1);
        }
        catch (IOException e) {
            Log.error("Could not read: " + mainTxt);
        }
        catch (NoSuchElementException e) {
            Log.error("Could not parse main component.");
        }
        return mainComponent;
    }

    private static String parseMainInstantiation(String mainTxt) {
        String mainInstantiation = "defaultInstantiation";
        try {
            Pattern p = Pattern.compile("Main-Component-Instantiation:\\s+(\\w+(?:\\.\\w+)*)\\s+(\\w+);");
            Stream<String> lines = Files.lines(Paths.get(mainTxt));
            mainInstantiation = lines.map(p::matcher)
                    .filter(Matcher::matches)
                    .findFirst().get().group(2);
        }
        catch (IOException e) {
            Log.error("Could not read: " + mainTxt);
        }
        catch (NoSuchElementException e) {
            Log.error("Could not parse main instantiation.");
        }
        return mainInstantiation;
    }

    private static ModelPath parseModelPath(String mainTxt) {
        ModelPath modelPath = new ModelPath();
       try {
            Pattern mpLine = Pattern.compile("Model-Paths:\\s+\\[(.*)\\]");
            Stream<String> lines = Files.lines(Paths.get(mainTxt));
            String mpString = lines.map(mpLine::matcher)
                    .filter(Matcher::matches)
                    .findFirst().get().group(1);

            Matcher mpEntry = Pattern.compile("((?:\\.|\\.\\.|/|\\w+)+)").matcher(mpString);

            Path mainTxtDir = Paths.get(mainTxt).getParent();
            while(mpEntry.find()) {
                modelPath.addEntry(Paths.get(mainTxtDir.toString() + File.separator + mpEntry.group()));
            }
        }
        catch (IOException e) {
            Log.error("Could not read: " + mainTxt);
        }
        catch (NoSuchElementException e) {
            Log.error("Could not parse main instantiation.");
        }
        return modelPath;
    }
}
