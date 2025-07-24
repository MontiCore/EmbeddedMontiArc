@echo off
cd /d %~dp0
java --class-path "target/libs/GDL-7.2.1-SNAPSHOT.jar;target/libs/GDL-cli.jar" de.monticore.lang.gdl.GDLTest %*
