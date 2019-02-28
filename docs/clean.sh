if [ "${PWD##*/}" = "docs" ] 
then 
    cd .. 
fi


echo Cleaning Unicorn
cd unicorn
make clean
cd ..

echo Cleaning Zydis
cd zydis
rm -r build
cd ..

echo Cleaning pe-parse
cd pe-parse
rm -r Release
rm -r Debug
cd ..

echo Cleaning hardware_emulator
cd hardware_emulator
rm -r Release
rm -r Debug
cd ..
