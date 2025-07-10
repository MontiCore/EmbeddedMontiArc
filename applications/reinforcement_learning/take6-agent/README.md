# 6 nimmt! Trainingsagent und Trainingsumgebung
Implementation Trainingsumgebung für 6 nimmt!

## Vorraussetzungen
- Java 17 oder neuer
- Maven 3.8.3 oder neuer
- swi Prolog

## Compilation
```bash
mvn package -s settings.xml
```

## Trainingsumgebung ausführen
###  Vorraussetzungen
- SWI-Prolog (Follow [official installation guide](https://www.swi-prolog.org/build/unix.html))
- Roscore
ROS Kinetic (Follow [official installation guide](http://wiki.ros.org/Installation/Ubuntu))

### Ausführung 
- Trainingsumgebung zum Training ausführen
```bash
roscore #start ROS master
java -cp "target/snimmt-environment-1.0-SNAPSHOT.jar" de.gdl.rl.environment.games.snimmt.SnimmtEnv --training
```
- Trainingsumgebung zum Spielen ausführen

- 
```bash
roscore #start ROS master
java -cp "target/snimmt-environment-1.0-SNAPSHOT.jar" de.gdl.rl.environment.games.snimmt.SnimmtEnv --gaming
```
- Trainingsumgebung zur Evaluation ausführen (hier 50 Spiele)
```bash
roscore #start ROS master
java -cp "target/snimmt-environment-1.0-SNAPSHOT.jar" de.gdl.rl.environment.games.snimmt.SnimmtEnv --evaluation 50
```
## 6nimmt-agent ausführen
### 1. EMADL-Toolchain installieren
  - Alle Vorraussetzungen für EMADL toolchain installieren. Vorraussetzungen werden [hier](doc/EMADL_SETUP.md) gelistet.
### 2. Agent compilieren
  - Run 
    ```
    ./install.sh
    ```
### 3. Training
  - roscore und Trainingsumgebung starten
  - Run 
    ```bash
    ./run_training.sh
    ```
### 4. Ausführung
  - roscore und SPielumgebung starten
  - Ausführbarer Agent im ```bin``` Ordner. 
  - Ausführen des Agenten mit folgendem Kommando:
    ```bash
    ./agent -executeOnDemand
    ```
