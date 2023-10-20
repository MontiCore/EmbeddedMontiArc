package de.monticore.lang.monticar.utilities.artifactcreator;

import de.monticore.lang.monticar.utilities.models.FileLocation;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarCreator;
import java.io.File;
import java.io.IOException;
import java.util.*;
import java.util.jar.Attributes;
import java.util.jar.Manifest;

public class ConfigCheckArtifactCreator extends ArtifactCreator {

    public static File createArtifact(StorageInformation storageInformation, String tmpOut) throws IOException {
        Manifest manifest = createManifest(storageInformation.getGroupId(), storageInformation.getArtifactId(), storageInformation.getVersion(), new Attributes());
        String jarFileName = createJarFileName(tmpOut, storageInformation.getArtifactId());
        List<FileLocation> fileLocations = getFileLocations(new File(tmpOut));
        if (fileLocations.isEmpty()) {
            throw new RuntimeException("No " + storageInformation.getArtifactId() + ".json file was found in " + tmpOut);
        }
        return JarCreator.createArtifact(jarFileName, manifest, fileLocations);
    }

    public static List<FileLocation> getFileLocations(File pathTmpOut) {
        List<FileLocation> configCheckLocations = new ArrayList<>();
        if (pathTmpOut.isDirectory()) {
            for (File file : Objects.requireNonNull(pathTmpOut.listFiles())) {
                if (file.getName().endsWith(".json")) {
                    FileLocation fileLocation = new FileLocation();
                    fileLocation.setSourceLocation(file.getAbsolutePath());
                    fileLocation.setJarLocation(file.getName());
                    configCheckLocations.add(fileLocation);
                }
            }
        }
        return configCheckLocations;
    }
}
