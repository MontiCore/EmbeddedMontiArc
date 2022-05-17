# GDL

SLE project Game Description Language

Execute Interpreter with CLI:
```bash
java --class-path "target/libs/GDL-7.1.0-SNAPSHOT.jar;target/libs/GDL-cli.jar" de.monticore.lang.gdl.GDLInterpreter "src/main/resources/example/Chess.gdl"
```

Execute Interpreter with Chess-GUI and CLI:
```bash
java --class-path "target/libs/GDL-7.1.0-SNAPSHOT.jar;target/libs/GDL-cli.jar" de.monticore.lang.gdl.GDLInterpreter "src/main/resources/example/Chess.gdl" -cg
```

Execute Interpreter with Chess-GUI and without CLI:
```bash
java --class-path "target/libs/GDL-7.1.0-SNAPSHOT.jar;target/libs/GDL-cli.jar" de.monticore.lang.gdl.GDLInterpreter "src/main/resources/example/Chess.gdl" -cg -nc
```

Load in jshell:
```bash
jshell --class-path "target/libs/GDL-7.1.0-SNAPSHOT.jar;target/libs/GDL-cli.jar"
```

Execute Interpreter on Ubuntu:
```bash
java -classpath "target/libs/GDL-7.1.0-SNAPSHOT.jar:target/libs/GDL-cli.jar" de.monticore.lang.gdl.GDLInterpreter "src/main/resources/example/TicTacToe.gdl"
```

Install .jar in local maven repository:

```bash
mvn install:install-file \
   -Dfile=target/libs/GDL-7.1.0-SNAPSHOT.jar \
   -DgroupId=de.monticore.lang.gdl \
   -DartifactId=lang.gdl \
   -Dversion=1.0 \
   -Dpackaging=jar \
   -DgeneratePom=true

mvn install:install-file \
   -Dfile=target/libs/GDL-cli.jar \
   -DgroupId=de.monticore.lang.gdl \
   -DartifactId=lang.gdl.cli \
   -Dversion=1.0 \
   -Dpackaging=jar \
   -DgeneratePom=true
```

