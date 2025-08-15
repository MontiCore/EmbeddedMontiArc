package de.monticore.lang.monticar.cnnarch.generator.util;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class SchemaApiUtilTest {

    @Test
    void createGetterMethodName() {
        assertEquals("get_optimizer_learning_rate",SchemaApiUtil.createGetterMethodName("optimizer", "learning_rate"));
    }
}