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
../docs/build_zydis.sh
cd ..

echo ""
echo "**************************************"
echo "       Making pe-parse Release"
echo "**************************************"
cd pe-parse
../docs/build.sh Release

echo ""
echo "**************************************"
echo "        Making pe-parse Debug"
echo "**************************************"
../docs/build.sh Debug
cd ..

echo ""
echo "**************************************"
echo "   Making hardware_emulator Release"
echo "**************************************"
cd hardware_emulator
../docs/build.sh Release

echo ""
echo "**************************************"
echo "    Making hardware_emulator Debug"
echo "**************************************"
../docs/build.sh Debug
cd ..
