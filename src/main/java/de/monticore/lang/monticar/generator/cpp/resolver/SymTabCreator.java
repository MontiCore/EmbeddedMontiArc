package de.monticore.lang.monticar.generator.cpp.resolver;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConstantPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.optimization.ThreadingOptimizer;
import de.monticore.lang.monticar.generator.order.nfp.TagBreakpointsTagSchema.TagBreakpointsTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagDelayTagSchema.TagDelayTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema.TagExecutionOrderTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagInitTagSchema.TagInitTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagMinMaxTagSchema.TagMinMaxTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagTableTagSchema.TagTableTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagThresholdTagSchema.TagThresholdTagSchema;
import de.monticore.lang.monticar.streamunits._symboltable.StreamUnitsLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;

import java.nio.file.Path;

public class SymTabCreator {

    private final Path[] modelPaths;

    public SymTabCreator(Path... modelPaths) {
        this.modelPaths = modelPaths;
    }

    public Scope createSymTab() {
        ConstantPortSymbol.resetLastID();
        MathConverter.resetIDs();
        ThreadingOptimizer.resetID();
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        EmbeddedMontiArcMathLanguage montiArcLanguage = new EmbeddedMontiArcMathLanguage();

        registerDefaultTags(montiArcLanguage);

        fam.addModelingLanguage(montiArcLanguage);
        fam.addModelingLanguage(new StreamUnitsLanguage());

        ModelPath mp = new ModelPath(modelPaths);
        LogConfig.init();
        return new GlobalScope(mp, fam);
    }

    private void registerDefaultTags(EmbeddedMontiArcMathLanguage montiArcLanguage) {
        TagMinMaxTagSchema.registerTagTypes(montiArcLanguage);
        TagTableTagSchema.registerTagTypes(montiArcLanguage);
        TagBreakpointsTagSchema.registerTagTypes(montiArcLanguage);
        TagExecutionOrderTagSchema.registerTagTypes(montiArcLanguage);
        TagInitTagSchema.registerTagTypes(montiArcLanguage);
        TagThresholdTagSchema.registerTagTypes(montiArcLanguage);
        TagDelayTagSchema.registerTagTypes(montiArcLanguage);
    }

}
