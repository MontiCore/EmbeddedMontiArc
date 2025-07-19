package de.monticore.lang.monticar.cnnarch.generator.configuration;

import org.junit.jupiter.api.DynamicTest;
import org.junit.jupiter.api.TestFactory;

import java.util.Arrays;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

class TemplateTest {

  @TestFactory
  Stream<DynamicTest> allTemplatePathsAreValid() {
    return Arrays.stream(Template.values())
        .map(template -> DynamicTest.dynamicTest(template.name(),
            () -> assertTrue(template.toPath().toFile().exists())));
  }

}