@rem (c) https://github.com/MontiCore/monticore  
cd "..\shared"
call variables.bat
cd "..\autopilot"
call generate.bat
call compile.bat
call start-simulation.bat
