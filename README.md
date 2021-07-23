# GDL

SLE project Game Description Language

Interpreter mit CLI ausführen:
```bash
java --class-path "target/libs/GDL-7.1.0-SNAPSHOT.jar;target/libs/GDL-cli.jar" de.monticore.lang.gdl.GDLInterpreter "src/main/resources/example/Chess.gdl"
```

Interpreter mit Schach-GUI und CLI ausführen:
```bash
java --class-path "target/libs/GDL-7.1.0-SNAPSHOT.jar;target/libs/GDL-cli.jar" de.monticore.lang.gdl.GDLInterpreter "src/main/resources/example/Chess.gdl" -cg
```

Interpreter mit Schach-GUI , ohne CLI ausführen:
```bash
java --class-path "target/libs/GDL-7.1.0-SNAPSHOT.jar;target/libs/GDL-cli.jar" de.monticore.lang.gdl.GDLInterpreter "src/main/resources/example/Chess.gdl" -cg -nc
```

In jshell laden:
```bash
jshell --class-path "target/libs/GDL-7.1.0-SNAPSHOT.jar;target/libs/GDL-cli.jar"
```
