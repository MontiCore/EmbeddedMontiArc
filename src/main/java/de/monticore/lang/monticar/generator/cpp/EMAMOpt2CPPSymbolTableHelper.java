/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguageFamiliy;
import de.monticore.lang.monticar.Utils;
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
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;

/**
 * Extends EMAMOptSymbolTableHelper
 * Additionally adds function to create tagging resolver
 *
 */
public class EMAMOpt2CPPSymbolTableHelper{

    private EMAMOpt2CPPSymbolTableHelper() {
    }

    protected static EMAMOpt2CPPSymbolTableHelper ourInstance;

    public static EMAMOpt2CPPSymbolTableHelper getInstance() {
        if (ourInstance == null)
            ourInstance = new EMAMOpt2CPPSymbolTableHelper();
        return ourInstance;
    }

    public Scope createSymTab(String... modelPath) {
        ModelingLanguageFamily fam = new EmbeddedMontiArcMathLanguageFamiliy();
        ModelPath mp = getModelPath(modelPath);
        GlobalScope scope = new GlobalScope(mp, fam);
        Utils.addBuiltInTypes(scope);
        LogConfig.init();
        return scope;
    }

    public ModelPath getModelPath(String... modelPath) {
        ModelPath mp = new ModelPath(new Path[0]);
        String[] var2 = modelPath;
        int var3 = modelPath.length;
        for (int var4 = 0; var4 < var3; ++var4) {
            String m = var2[var4];
            mp.addEntry(Paths.get(m));
        }
        return mp;
    }

    public TaggingResolver createSymTabAndTaggingResolver(String... modelPath) {
        MathConverter.resetIDs();
        ThreadingOptimizer.resetID();
        MathOptimizer.resetIDs();
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
}
