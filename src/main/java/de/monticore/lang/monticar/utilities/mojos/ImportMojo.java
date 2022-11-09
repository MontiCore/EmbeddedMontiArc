package de.monticore.lang.monticar.utilities.mojos;

import de.monticore.lang.monticar.utilities.artifactinmporter.ArtifactImporter;
import org.apache.commons.lang3.StringUtils;
import org.apache.maven.model.Dependency;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.LifecyclePhase;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;
import org.apache.maven.project.MavenProject;
import org.apache.maven.shared.invoker.MavenInvocationException;

import java.io.File;
import java.util.List;

/**
 *  Imports an EMADL project JAR to a local project.
 *
 */
@Mojo(name = "import", defaultPhase = LifecyclePhase.INSTALL)
public class ImportMojo extends BaseMojo {

  @Parameter(defaultValue = "imported-resources")
  private File targetPath;

  @Override
  public void execute() throws MojoExecutionException, MojoFailureException {
    MavenProject mavenProject = (MavenProject) this.getPluginContext().get("project");
    List<Dependency> dependencies = mavenProject.getDependencies();

    for (Dependency dependency : dependencies) {
      if (StringUtils.equals("emadl", dependency.getClassifier())) {
        try {
          ArtifactImporter.importArtifact(dependency, targetPath);
        }
        catch (MavenInvocationException e) {
          e.printStackTrace();
        }
      }
    }

  }

}
