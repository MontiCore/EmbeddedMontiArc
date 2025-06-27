package de.monticore.lang.monticar.utilities.artifactdeployer;

import de.monticore.lang.monticar.utilities.models.StorageInformation;
import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import org.apache.maven.model.Dependency;
import org.apache.maven.model.DeploymentRepository;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class POMGenerator {
    public static File generatePOM(StorageInformation storageInformation, DeploymentRepository repository, List<Dependency> dependencies) throws IOException {
        Configuration cfg = new Configuration(Configuration.VERSION_2_3_29);
        cfg.setClassForTemplateLoading(POMGenerator.class, "/templates");

        Map<String, Object> variables = new HashMap<>();
        variables.put("storage", storageInformation);
        variables.put("repository", repository);
        variables.put("dependencies", dependencies);

        Template template = cfg.getTemplate("pom.ftl");

        File tmpFile = File.createTempFile("emadl-maven-plugin-", null);
        tmpFile.deleteOnExit();

        try {
            template.process(variables, new FileWriter(tmpFile));
        } catch (TemplateException e){
            e.printStackTrace();
            throw new RuntimeException("Could not generate pom.xml. Exception during template rendering.");
        }

        return tmpFile;
    }
}
