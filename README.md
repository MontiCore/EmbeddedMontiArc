# GDL

SLE project Game Description Language

## GDL Models

- [TicTacToe](src/main/resources/example/TicTacToe.gdl)
- [Chess](src/main/resources/example/Chess.gdl)
- [Doppelkopf](src/main/resources/example/Doppelkopf.gdl)

## Usage

Launch GDL Interpreter with CLI (Windows PowerShell / Bash):
```bash
./gdl-cli "src/main/resources/example/Chess.gdl"
```

Command line Options:
```
-cg, --chess-gui        Start with a Chess GUI
-dg, --doppelkopf-gui   Start with a Doppelkopf GUI
-nc, --no-cli           Disable the CLI
-dm, --debug-mode       Enable the debug mode
-mr, --manual-random    Enable manual control over the random role
-st, --show-times       Profile the runtime of some functions
-wt, --with-types       Create features for strongly typed models
```

---

Install .jar in local maven repository:
```bash
mvn install:install-file \
   -Dfile=target/libs/GDL-7.2.0-SNAPSHOT.jar \
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

