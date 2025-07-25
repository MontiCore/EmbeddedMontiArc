package de.monticore.lang.monticar.utilities.utils;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.monticore.lang.monticar.utilities.models.Dataset;
import de.monticore.lang.monticar.utilities.models.FileLocation;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;

import static de.monticore.lang.monticar.utilities.artifactcreator.DatasetArtifactCreator.getFileExtension;

public class JsonCreator {
    public static void createJSONMetadata(Map<String, FileLocation> fileLocations, List<Dataset> datasets){
        for(Dataset dataset: datasets){
            FileLocation fileLocation = fileLocations.get(dataset.getId());

            if(dataset.getFiletype() == null){
                dataset.setFiletype(getFileExtension(dataset.getPath()));
            }

            ObjectMapper mapper = new ObjectMapper();

            try {
                mapper.writeValue(new File(fileLocation.getPropertiesLocation()), dataset);
            } catch (IOException e){
                e.printStackTrace();
            }
        }

    }
}
