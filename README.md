# GDL

SLE project Game Description Language

In jshell ausführen:
```bash
jshell --class-path "target/libs/GDL-7.1.0-SNAPSHOT.jar;target/libs/GDL-cli.jar"
```
```java
import de.monticore.lang.gdl.*;
GDLTool tool = new GDLTool();
tool.main(new String[]{"test.gdl"});
```
Oder einfach:
```bash
java --class-path "target/libs/GDL-7.1.0-SNAPSHOT.jar;target/libs/GDL-cli.jar" de.monticore.lang.gdl.Interpreter2 "src/main/resources/example/Chess.gdl"
```
