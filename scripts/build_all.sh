if [ "${PWD##*/}" = "docs" ] 
then
    cd ..
fi

echo ""
echo "**************************************"
echo "           Making Unicorn"
echo "**************************************"
cd unicorn
./make.sh
cd ..

echo ""
echo "**************************************"
echo "             Making Zydis"
echo "**************************************"
cd zydis
./build.sh
cd ..

echo ""
echo "**************************************"
echo "       Making pe-parse Release"
echo "**************************************"
cd pe-parse
../scripts/build.sh Release

echo ""
echo "**************************************"
echo "        Making pe-parse Debug"
echo "**************************************"
../scripts/build.sh Debug
cd ..

echo ""
echo "**************************************"
echo "   Making hardware_emulator Release"
echo "**************************************"
cd hardware_emulator
../scripts/build.sh Release

echo ""
echo "**************************************"
echo "    Making hardware_emulator Debug"
echo "**************************************"
../scripts/build.sh Debug
cd ..
