This maven plugin performs stream tests on EMAM components.

### Usage:
Add this to your pom.xml
```
<!-- EMAM Streamtest -->
<build>
    <plugins>
    ...
    <plugin>
        <groupId>de.monticore.lang.montiarc.utilities</groupId>
        <artifactId>maven-streamtest</artifactId>
        <version>0.0.1-SNAPSHOT</version>
        <configuration>
            <pathMain>./src/main/emam</pathMain>
            <pathTest>./src/test/emam</pathTest>
            <pathTmpOut>./target/tmp</pathTmpOut>
            <gppPathToArmadilloH>/usr/local/Cellar/armadillo/8.500.1/include</gppPathToArmadilloH>
            <wrapperTestExtension>_TestWrapper</wrapperTestExtension>
            <gpp>g++</gpp>
        </configuration>
    
        <executions>
            <execution>
                <phase>test</phase>
                <goals>
                    <goal>emam-streamtest</goal>
                </goals>
            </execution>
        </executions>
    </plugin>
    ...
    </plugins>
</build>
```

