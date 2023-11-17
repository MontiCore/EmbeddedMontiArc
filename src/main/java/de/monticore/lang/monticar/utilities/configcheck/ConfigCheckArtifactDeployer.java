package de.monticore.lang.monticar.utilities.configcheck;

import de.monticore.lang.monticar.utilities.artifactdeployer.ArtifactDeployer;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarClassifierEnum;
import org.apache.maven.model.DeploymentRepository;
import org.apache.maven.shared.invoker.MavenInvocationException;

import java.io.File;
import java.io.IOException;

public class ConfigCheckArtifactDeployer {
    public static void deployArtifact(StorageInformation storageInformation, DeploymentRepository repository, File settingsFile) {
        File jarFile;
        try {
            System.out.println(String.format("STARTING creating Jar for config-check %s", storageInformation.getPath()));
            jarFile = ConfigCheckArtifactCreator.createArtifact(storageInformation, storageInformation.getPath().getPath());
            System.out.println("FINISHED creating Jar for config-check");

            ArtifactDeployer.deployArtifact(jarFile.getAbsolutePath(), storageInformation, repository, JarClassifierEnum.EMPTY, settingsFile);
        }
        catch (IOException | MavenInvocationException e) {
            e.printStackTrace();
        }
    }
}
