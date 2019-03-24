if [ "${PWD##*/}" = "scripts" ] 
then
    cd ..
fi
cd samples

echo "**************************************"
echo "        Updating simple sample"
echo "**************************************"
cd simple
make
cp "sample_simple.so" "../../hardware_emulator/bin/"
cd ..


echo "**************************************"
echo "     Updating funccalling sample"
echo "**************************************"
cd funccalling
make
cp "sample_functioncalling.so" "../../hardware_emulator/bin/"
cd ..

echo "**************************************"
echo "      Updating syscall sample"
echo "**************************************"
cd syscall_so
make
cp "sample_syscall.so" "../../hardware_emulator/bin/"
cd ..

cd ..
