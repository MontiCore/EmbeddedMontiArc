
For more information see Wiki:
[Link](./wikis/home)



```
    <plugins>
        ...
        <plugin>
            <groupId>de.monticore.lang.monticar.utilities</groupId>
            <artifactId>maven-streamtest</artifactId>
            <version>0.0.3-SNAPSHOT</version>
            <configuration>
                <pathMain>./src/main/emam</pathMain>
                <pathTest>./src/test/emam</pathTest>
                <pathTmpOut>./target/tmp</pathTmpOut>
                <generator>VS2017</generator>
            </configuration>
    
            <executions>
                <execution>
                    <phase>test</phase>
                    <goals>
                        <goal>streamtest-execute</goal>
                    </goals>
                </execution>
            </executions>
        </plugin>
        ...
    </plugins>
```
