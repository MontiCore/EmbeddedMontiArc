@rem (c) https://github.com/MontiCore/monticore  

subst N: .
cd /D N:\
cd native
call variables.bat
call compileTestsArmadilloBackendOpenBLAS.bat %PROJECT_ROOT%%1 %PROJECT_ROOT%%2
