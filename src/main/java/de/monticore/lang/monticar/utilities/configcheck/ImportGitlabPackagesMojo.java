package de.monticore.lang.monticar.utilities.configcheck;

import de.monticore.lang.monticar.utilities.mojos.TrainingConfigMojo;
import org.apache.maven.model.Dependency;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.shared.invoker.MavenInvocationException;

import java.io.File;

@Mojo(name = "import-gitlab-packages")
public class ImportGitlabPackagesMojo extends TrainingConfigMojo {
    @Override
    public void execute() throws MojoExecutionException, MojoFailureException {
        String targetPath = getPathTmpOut() + "/runConfigurations";
        ConfigCheck configCheck = new ConfigCheck(getTrainingConfig(), getPathTmpOut());
        getLog().info("STARTING importing packages from gitlab");
        Dependency dependency = configCheck.getDependency(getMavenProject().getVersion());
        try {
            ConfigCheckArtifactImporter.importArtifact(dependency, new File(targetPath));
        } catch (MavenInvocationException e) {
            e.printStackTrace();
        }
        getLog().info("FINISHED importing packages from gitlab");
    }
}
