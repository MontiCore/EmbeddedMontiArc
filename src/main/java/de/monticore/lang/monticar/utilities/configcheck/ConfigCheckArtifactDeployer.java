package de.monticore.lang.monticar.utilities.configcheck;

import de.monticore.lang.monticar.utilities.artifactdeployer.ArtifactDeployer;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarClassifierEnum;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.model.DeploymentRepository;
import org.apache.maven.shared.invoker.MavenInvocationException;
import java.io.File;
import java.io.IOException;

public class ConfigCheckArtifactDeployer {
    public static void deployArtifact(StorageInformation storageInformation, DeploymentRepository repository, File settingsFile) {
        File jarFile;
        try {
            Log.info(String.format("STARTING creating Jar for config-check %s", storageInformation.getPath()), "[ConfigCheck]");
            jarFile = ConfigCheckArtifactCreator.createArtifact(storageInformation, storageInformation.getPath().getPath());
            Log.info("FINISHED creating Jar for config-check", "[ConfigCheck]");
            Log.info(String.format("[ConfigCheck] Deploying artifact '%s'", jarFile.getName()), "[ConfigCheck]");
            ArtifactDeployer.deployArtifact(jarFile.getAbsolutePath(), storageInformation, repository, JarClassifierEnum.EMPTY, settingsFile);
        }
        catch (IOException | MavenInvocationException e) {
            e.printStackTrace();
        }
    }
}
