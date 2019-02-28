@echo off
IF "%1"=="" ( SET TARGET="Release" ) ELSE ( SET TARGET=%1 )
echo Building target %TARGET%
msbuild build\elf-parser.sln /m /p:Configuration=%TARGET% /p:Platform=x64