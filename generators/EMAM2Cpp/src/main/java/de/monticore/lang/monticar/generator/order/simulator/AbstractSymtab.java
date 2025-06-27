/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.simulator;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.embeddedmontiarc.helper.ConstantPortHelper;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.EventLanguage;
import de.monticore.lang.monticar.enumlang._symboltable.EnumLangLanguage;
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

import java.nio.file.Paths;
import java.util.Arrays;

/**
 * Common methods for symboltable tests
 *
 */
public class AbstractSymtab {
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
        EmbeddedMontiArcMathLanguage montiArcLanguage = new EmbeddedMontiArcMathLanguage();

        fam.addModelingLanguage(montiArcLanguage);
        fam.addModelingLanguage(new StreamUnitsLanguage());
        fam.addModelingLanguage(new StructLanguage());
        fam.addModelingLanguage(new EnumLangLanguage());
        fam.addModelingLanguage(new EventLanguage());
        final ModelPath mp = new ModelPath();
        for (String m : modelPath) {
            mp.addEntry(Paths.get(m));
        }
        LogConfig.init();//TODO comment for debug output

        GlobalScope scope = new GlobalScope(mp, fam);
        de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);
        return scope;
    }
}
