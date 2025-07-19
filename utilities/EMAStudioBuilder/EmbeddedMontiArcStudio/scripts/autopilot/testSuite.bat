@rem (c) https://github.com/MontiCore/monticore  
call generateTests %1
call compileTests
call executeTest %1
