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
import de.monticore.lang.monticar.utilities.models.Constants;
import de.monticore.lang.monticar.utilities.models.Repository;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import org.apache.maven.execution.MavenSession;
import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugin.BuildPluginManager;
import org.apache.maven.plugins.annotations.Component;
import org.apache.maven.plugins.annotations.Parameter;
import org.apache.maven.project.MavenProject;
import org.eclipse.aether.RepositorySystem;
import org.eclipse.aether.RepositorySystemSession;
import org.eclipse.aether.artifact.Artifact;
import org.eclipse.aether.artifact.DefaultArtifact;
import org.eclipse.aether.repository.RemoteRepository;
import org.eclipse.aether.resolution.VersionRangeRequest;
import org.eclipse.aether.resolution.VersionRangeResolutionException;
import org.eclipse.aether.resolution.VersionRangeResult;

import java.io.File;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;

public abstract class BaseMojo extends AbstractMojo {

  @Component
  private RepositorySystem repositorySystem;

  @Component
  private MavenProject mavenProject;

  @Component
  private MavenSession mavenSession;

  @Component
  private BuildPluginManager pluginManager;

  @Parameter( defaultValue = "${repositorySystemSession}", readonly = true )
  private RepositorySystemSession repositorySystemSession;

  @Parameter
  private Repository repository;

  @Parameter
  private TrainingConfiguration training;

  @Parameter(property = "pathMain",defaultValue = "src/main/emadl")
  private String pathMain;

  @Parameter(property = "pathTest",defaultValue = "src/main/emadl")
  private String pathTest;

  @Parameter(property = "pathTmpOut", defaultValue = "target/tmp")
  private String pathTmpOut;

  private Scope myScope;

  private TaggingResolver myTaggingResolver;

  private RemoteRepository remoteRepository;

  public void mkTmpDir() {
    this.mkdir(getPathTmpOut());
  }

  private void mkdir(String path) {
    try {
      File tmpOut = Paths.get(path).toFile();
      if(!tmpOut.exists()){
        tmpOut.mkdirs();
      }
    }catch (Exception ex){
      ex.printStackTrace();
    }
  }

  public MavenProject getMavenProject() {
    return mavenProject;
  }

  public MavenSession getMavenSession() {
    return mavenSession;
  }

  public BuildPluginManager getPluginManager() {
    return pluginManager;
  }

  public TrainingConfiguration getTrainingConfig() {
    return training;
  }

  public String getPathMain() {
    return pathMain;
  }

  public String getPathTest() {
    return pathTest;
  }

  public String getPathTmpOut() {
    return pathTmpOut;
  }

  public String getPathTmpOutCPP(){
    return Paths.get(this.pathTmpOut, "cpp/").toString();
  }

  public String getPathTmpOutEMAM(){
    return Paths.get(this.pathTmpOut, "emam/").toString();
  }

  public String getPathTmpOutEMADL(){
    return Paths.get(this.pathTmpOut,"emadl/").toString();
  }

  public String getPathTmpOutBUILD() {
    return Paths.get(this.getPathTmpOut(), "build/").toString();
  }

  public Repository getRepository() {
    return repository;
  }

  public RepositorySystem getRepositorySystem() {
    return repositorySystem;
  }

  public RepositorySystemSession getRepositorySystemSession() {
    return repositorySystemSession;
  }

  public RemoteRepository getRemoteRepository() {
    if (remoteRepository != null) {
      return remoteRepository;
    }

    remoteRepository = new RemoteRepository.Builder(repository.getId(), "default", repository.getUrl().getPath()).build();
    return remoteRepository;
  }

  public int getNewestVersion(StorageInformation storageInformation) {
    if (storageInformation.getVersion() != null) {
      return storageInformation.getVersion();
    }

    Artifact artifact = new DefaultArtifact( String.format("%s:%s:[1,)" , storageInformation.getGroupId(), storageInformation.getArtifactId()));

    VersionRangeRequest rangeRequest = new VersionRangeRequest();
    rangeRequest.setArtifact( artifact );
    rangeRequest.setRepositories(Collections.singletonList(getRemoteRepository()));

    int newestVersion;
    try {
      VersionRangeResult rangeResult = repositorySystem.resolveVersionRange( repositorySystemSession, rangeRequest );
      newestVersion = rangeResult.getHighestVersion() != null ? Integer.parseInt(rangeResult.getHighestVersion().toString()) : Constants.INITIAL_VERSION - 1;
      return ++newestVersion;
    }
    catch (VersionRangeResolutionException e) {
      e.printStackTrace();
    }

    return Constants.INITIAL_VERSION;
  }

  public Scope getScope(){
    if(myScope == null) {
      ModelingLanguageFamily fam = new ModelingLanguageFamily();
      fam.addModelingLanguage(new EmbeddedMontiArcMathLanguage());
      fam.addModelingLanguage(new StreamUnitsLanguage());
      fam.addModelingLanguage(new StructLanguage());
      fam.addModelingLanguage(new EnumLangLanguage());
      fam.addModelingLanguage(new EMADLLanguage());
      final ModelPath mp_main = new ModelPath();

      mp_main.addEntry(Paths.get(this.getPathMain()));
      if (!this.getPathMain().equals(this.getPathTest())) {
        mp_main.addEntry(Paths.get(this.getPathTest()));
      }

      GlobalScope gs = new GlobalScope(mp_main, fam);
      de.monticore.lang.monticar.Utils.addBuiltInTypes(gs);

      ArrayList<String> col = new ArrayList<String>();

      col.add(this.getPathMain());
      if (!this.getPathMain().equals(this.getPathTest())) {
        col.add(this.getPathTest());
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
  protected TaggingResolver getTaggingResolver(){
    if(this.myTaggingResolver == null){
      this.getScope();
    }
    return this.myTaggingResolver;
  }



}
