/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.grammars.options.visitor;

import com.google.inject.Guice;
import de.monticore.lang.monticar.sol.grammars.options.OptionsModule;
import de.monticore.lang.monticar.sol.grammars.options.optionstest._parser.OptionsTestParser;
import org.apache.commons.io.FileUtils;
import org.json.JSONObject;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class OptionsSerializerImplTests {
    OptionsSerializer serializer = Guice.createInjector(new OptionsModule()).getInstance(OptionsSerializer.class);

    @Test
    @SuppressWarnings("OptionalGetWithoutIsPresent")
    void testSerialize() throws IOException {
        OptionsTestParser parser = new OptionsTestParser();
        File visitorTestFile = Paths.get("src/test/resources/visitor/Visitor.test").toFile();
        File visitorFile = Paths.get("src/test/resources/visitor/Visitor.json").toFile();
        JSONObject serializedOption = new JSONObject(FileUtils.readFileToString(visitorFile, "UTF-8"));
        JSONObject serializedTestOption = serializer.serialize(parser.parse(visitorTestFile.getPath()).get());

        assertEquals(serializedOption.toString(), serializedTestOption.toString(), "Serialized options do not match.");
    }
}
