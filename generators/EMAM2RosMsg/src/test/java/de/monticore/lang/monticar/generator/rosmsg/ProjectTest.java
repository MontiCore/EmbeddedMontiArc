/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.rosmsg;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import static org.junit.Assert.assertNotNull;

public class ProjectTest extends AbstractSymtabTest{

    @Test
    public void testRosProjectGeneration() throws IOException {
        Scope symtab = createSymTab("src/test/resources/");
        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.basicStructComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        GeneratorRosMsg generatorRosMsg = new GeneratorRosMsg();
        generatorRosMsg.setTarget("target/projects/ros/basicStructComp","struct_msgs");
        List<File> files = generatorRosMsg.generateProject(component);

        checkFilesAreEqual(files, Paths.get("target/projects/ros/basicStructComp"),"projects/ros/basicStructComp/");
    }


    @Test
    public void testRos2ProjectGeneration() throws IOException {
        Scope symtab = createSymTab("src/test/resources/");
        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.basicStructComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        GeneratorRosMsg generatorRosMsg = new GeneratorRosMsg();
        generatorRosMsg.setTarget("target/projects/ros2/basicStructComp","struct_msgs");
        generatorRosMsg.setRos2mode(true);
        List<File> files = generatorRosMsg.generateProject(component);

        checkFilesAreEqual(files, Paths.get("target/projects/ros2/basicStructComp"),"projects/ros2/basicStructComp/");
    }

}
