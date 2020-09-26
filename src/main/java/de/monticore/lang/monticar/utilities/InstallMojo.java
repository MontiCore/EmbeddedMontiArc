package de.monticore.lang.monticar.utilities;

import de.monticore.lang.monticar.utilities.artifactinstaller.ArtifactInstaller;
import org.apache.commons.lang3.StringUtils;
import org.apache.maven.model.Dependency;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.project.MavenProject;
import org.apache.maven.shared.invoker.MavenInvocationException;

import java.util.List;

@Mojo(name = "install")
public class InstallMojo extends BaseMojo {

  @Override
  public void execute() throws MojoExecutionException, MojoFailureException {
    MavenProject mavenProject = (MavenProject) this.getPluginContext().get("project");
    List<Dependency> dependencies = mavenProject.getDependencies();

    for (Dependency dependency: dependencies) {
      if (StringUtils.equals("dataset", dependency.getClassifier()) || StringUtils.equals("emadl", dependency.getClassifier())) {
        try {
          ArtifactInstaller.installArtifact(dependency);
        }
        catch (MavenInvocationException e) {
          e.printStackTrace();
        }
      }

      System.out.println(dependencies);
    }


  }

}
