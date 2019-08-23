@rem (c) https://github.com/MontiCore/monticore  
if not exist "%USERPROFILE%\visualization-emam" mkdir "%USERPROFILE%\visualization-emam"

robocopy "%HOME%\visualization-emam\math-pretty-printer" "%USERPROFILE%\visualization-emam\math-pretty-printer"
robocopy "%HOME%\visualization-emam\visualisation" "%USERPROFILE%\visualization-emam\visualisation"
