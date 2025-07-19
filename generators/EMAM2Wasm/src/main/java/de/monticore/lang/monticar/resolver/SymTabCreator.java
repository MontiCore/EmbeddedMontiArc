/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.resolver;

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
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class SymTabCreator {

  private final Path[] modelPaths;

  public SymTabCreator(Path... modelPaths) {
    this.modelPaths = modelPaths;
  }

  public TaggingResolver createSymTabAndTaggingResolver() {
    Scope symtab = createSymTab();
    List<String> list = Arrays.stream(modelPaths).map(Path::toString)
        .collect(Collectors.toList());
    TaggingResolver taggingResolver = new TaggingResolver(symtab, list);
    registerDefaultTags(taggingResolver);
    return taggingResolver;
  }

  public Scope createSymTab() {
    ConstantPortSymbol.resetLastID();
    MathConverter.resetIDs();
    ThreadingOptimizer.resetID();
    ModelingLanguageFamily fam = new ModelingLanguageFamily();
    EmbeddedMontiArcMathLanguage montiArcLanguage = new EmbeddedMontiArcMathLanguage();

    fam.addModelingLanguage(montiArcLanguage);
    fam.addModelingLanguage(new StreamUnitsLanguage());

    ModelPath mp = new ModelPath(modelPaths);
    LogConfig.init();
    GlobalScope scope = new GlobalScope(mp, fam);
    de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);
    return scope;
  }

  private void registerDefaultTags(TaggingResolver tagginResolver) {
    TagMinMaxTagSchema.registerTagTypes(tagginResolver);
    TagTableTagSchema.registerTagTypes(tagginResolver);
    TagBreakpointsTagSchema.registerTagTypes(tagginResolver);
    TagExecutionOrderTagSchema.registerTagTypes(tagginResolver);
    TagInitTagSchema.registerTagTypes(tagginResolver);
    TagThresholdTagSchema.registerTagTypes(tagginResolver);
    TagDelayTagSchema.registerTagTypes(tagginResolver);
  }

}
