package de.monticore.lang.monticar.utilities.mojos;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.monticar.emadl._symboltable.EMADLLanguage;
import de.monticore.lang.monticar.enumlang._symboltable.EnumLangLanguage;
import de.monticore.lang.monticar.generator.order.nfp.TagBreakpointsTagSchema.TagBreakpointsTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagDelayTagSchema.TagDelayTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema.TagExecutionOrderTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagInitTagSchema.TagInitTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagMinMaxTagSchema.TagMinMaxTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagTableTagSchema.TagTableTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagThresholdTagSchema.TagThresholdTagSchema;
import de.monticore.lang.monticar.streamunits._symboltable.StreamUnitsLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import org.apache.maven.plugins.annotations.Parameter;

import java.nio.file.Paths;
import java.util.ArrayList;

public abstract class TrainingConfigMojo extends BaseMojo {

  @Parameter
  private TrainingConfiguration training;

  private Scope myScope;

  private TaggingResolver myTaggingResolver;

  public TrainingConfiguration getTrainingConfig() {
    return training;
  }

  public Scope getScope() {
    if (myScope == null) {
      ModelingLanguageFamily fam = new ModelingLanguageFamily();
      fam.addModelingLanguage(new EmbeddedMontiArcMathLanguage());
      fam.addModelingLanguage(new StreamUnitsLanguage());
      fam.addModelingLanguage(new StructLanguage());
      fam.addModelingLanguage(new EnumLangLanguage());
      fam.addModelingLanguage(new EMADLLanguage());
      final ModelPath mp_main = new ModelPath();

      String pathToProject = this.getTrainingConfig().getPathToProject().getAbsolutePath();
      String pathToTest = this.getTrainingConfig().getPathToTest().getAbsolutePath();

      mp_main.addEntry(Paths.get(pathToProject));
      if (!pathToProject.equals(pathToTest)) {
        mp_main.addEntry(Paths.get(pathToTest));
      }

      GlobalScope gs = new GlobalScope(mp_main, fam);
      de.monticore.lang.monticar.Utils.addBuiltInTypes(gs);

      ArrayList<String> col = new ArrayList<String>();

      col.add(pathToProject);
      if (!pathToProject.equals(pathToTest)) {
        col.add(pathToTest);
      }

      this.myTaggingResolver = new TaggingResolver(gs, col);
      TagMinMaxTagSchema.registerTagTypes(this.myTaggingResolver);
      TagTableTagSchema.registerTagTypes(this.myTaggingResolver);
      TagBreakpointsTagSchema.registerTagTypes(this.myTaggingResolver);
      TagExecutionOrderTagSchema.registerTagTypes(this.myTaggingResolver);
      TagInitTagSchema.registerTagTypes(this.myTaggingResolver);
      TagThresholdTagSchema.registerTagTypes(this.myTaggingResolver);
      TagDelayTagSchema.registerTagTypes(this.myTaggingResolver);
      myScope = gs;
    }
    return myScope;
  }

  public TaggingResolver getTaggingResolver() {
    if (this.myTaggingResolver == null) {
      this.getScope();
    }
    return this.myTaggingResolver;
  }



}
