function quitOnError {
    if [ "$?" -ne 0 ]
    then
        echo Failed!
        exit 1
    fi
}

cd target/generated-sources-cmake/
quitOnError

for d in */ ; do
    echo Trying to build "$d"
    echo creating dir "$d"/build
    mkdir "$d"/build
    quitOnError
    echo CMake
    cmake -B"$d"/build/ -H"$d"/src/
    quitOnError
    echo make
    make -C "$d"/build/
    quitOnError
done

echo success!
exit 0