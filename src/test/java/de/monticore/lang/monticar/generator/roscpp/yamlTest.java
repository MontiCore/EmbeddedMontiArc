package de.monticore.lang.monticar.generator.roscpp;

import com.esotericsoftware.yamlbeans.YamlException;
import de.monticar.lang.monticar.generator.python.RosTag;
import de.monticar.lang.monticar.generator.python.TagReader;
import org.junit.Test;

import java.io.FileNotFoundException;
import java.util.List;

import static org.junit.Assert.assertTrue;

public class yamlTest {

    @Test
    public void parseYamlTest() throws FileNotFoundException, YamlException {
        TagReader<RosTag> reader = new TagReader<>();
        List<RosTag> tags = reader.readYAML("src/test/resources/tests/config/config.yaml");

        assertTrue(tags.size() == 2);
    }

}
