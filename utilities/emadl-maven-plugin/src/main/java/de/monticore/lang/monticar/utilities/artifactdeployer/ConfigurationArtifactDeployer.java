package de.monticore.lang.monticar.utilities.artifactdeployer;

import de.monticore.lang.monticar.utilities.artifactcreator.ConfigurationArtifactCreator;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarClassifierEnum;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.model.DeploymentRepository;
import org.apache.maven.shared.invoker.MavenInvocationException;
import java.io.File;
import java.io.IOException;

public class ConfigurationArtifactDeployer extends ArtifactDeployer {
    public static void deployArtifact(StorageInformation storageInformation, DeploymentRepository repository, File settingsFile) {
        File jarFile;
        try {
            Log.info(String.format("STARTING creating Jar for configuration archive %s", storageInformation.getPath()), ConfigurationArtifactDeployer.class.getName());
            jarFile = ConfigurationArtifactCreator.createArtifact(storageInformation, storageInformation.getPath().getPath());
            Log.info("FINISHED creating Jar for configuration archive", ConfigurationArtifactDeployer.class.getName());
            deployArtifact(jarFile.getAbsolutePath(), storageInformation, repository, JarClassifierEnum.EMPTY, settingsFile);
        }
        catch (IOException | MavenInvocationException e) {
            e.printStackTrace();
        }
    }
}
