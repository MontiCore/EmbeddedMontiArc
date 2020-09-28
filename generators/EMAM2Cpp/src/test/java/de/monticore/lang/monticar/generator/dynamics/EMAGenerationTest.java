/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.dynamics;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.helper.ConstantPortHelper;
import de.monticore.lang.monticar.enumlang._symboltable.EnumLangLanguage;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.optimization.MathOptimizer;
import de.monticore.lang.monticar.generator.optimization.ThreadingOptimizer;
import de.monticore.lang.monticar.generator.order.nfp.TagBreakpointsTagSchema.TagBreakpointsTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagDelayTagSchema.TagDelayTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema.TagExecutionOrderTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagInitTagSchema.TagInitTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagMinMaxTagSchema.TagMinMaxTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagTableTagSchema.TagTableTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagThresholdTagSchema.TagThresholdTagSchema;
import de.monticore.lang.monticar.streamunits._symboltable.StreamUnitsLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.FixMethodOrder;
import org.junit.Test;
import org.junit.runners.MethodSorters;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.assertNotNull;



public class EMAGenerationTest {

    public static TaggingResolver createSymTabAndTaggingResolver(String... modelPath) {
        Scope scope = createSymTab(modelPath);
        TaggingResolver tagging = new TaggingResolver(scope, Arrays.asList(modelPath));
        TagMinMaxTagSchema.registerTagTypes(tagging);
        TagTableTagSchema.registerTagTypes(tagging);
        TagBreakpointsTagSchema.registerTagTypes(tagging);
        TagExecutionOrderTagSchema.registerTagTypes(tagging);
        TagInitTagSchema.registerTagTypes(tagging);
        TagThresholdTagSchema.registerTagTypes(tagging);
        TagDelayTagSchema.registerTagTypes(tagging);
        return tagging;
    }

    public static Scope createSymTab(String... modelPath) {
//        ConstantPortSymbol.resetLastID();
        ConstantPortHelper.resetLastID();
        MathConverter.resetIDs();
        ThreadingOptimizer.resetID();
        MathOptimizer.resetIDs();
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        EmbeddedMontiArcLanguage montiArcLanguage = new EmbeddedMontiArcLanguage();

        fam.addModelingLanguage(montiArcLanguage);
        fam.addModelingLanguage(new StreamUnitsLanguage());
        fam.addModelingLanguage(new StructLanguage());
        fam.addModelingLanguage(new EnumLangLanguage());
        final ModelPath mp = new ModelPath();
        for (String m : modelPath) {
            mp.addEntry(Paths.get(m));
        }
        LogConfig.init();//TODO comment for debug output

        GlobalScope scope = new GlobalScope(mp, fam);
        de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);
        return scope;
    }


    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }


    @Test
    public void Test_01_PassThrough() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/dynamics");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("ema.passThrough", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/dynamics/ema/test01");
        generatorCPP.setGenerateCMake(true);
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
//        files.stream().forEach(f -> System.out.println("Generated: "+f.getName()));
    }


    @Test
    public void Test_02_Components() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/dynamics");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("ema.components", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/dynamics/ema/test02");
        generatorCPP.setGenerateCMake(true);
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
//        files.stream().forEach(f -> System.out.println("Generated: "+f.getName()));
    }
}
