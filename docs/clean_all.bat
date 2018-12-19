@echo off

cd pe-parse
rmdir /S /Q build
cd ..

cd unicorn
rmdir /S /Q build
cd ..

cd zydis
rmdir /S /Q build
cd ..


cd hardware_emulator
rmdir /S /Q build
cd ..