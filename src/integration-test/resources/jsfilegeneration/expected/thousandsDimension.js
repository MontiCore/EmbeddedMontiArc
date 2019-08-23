/* (c) https://github.com/MontiCore/monticore */
var Module = {
    'print': function (text) {
        console.log('stdout: ' + text)
    },
    'printErr': function (text) {
        console.log('stderr: ' + text)
    },
    onRuntimeInitialized: function () {
        Module.init();
    }
};

function init() {
    Module.init();
}

function execute() {
    Module.execute();
}

function getOutThousandsMatrixArray() {
    return math.format(Module.getOutThousandsMatrixArray(), {notation: 'fixed'}).concat(" m");
}

function setInThousandsMatrixArray(_inThousandsMatrixArray) {
    var value = math.eval(_inThousandsMatrixArray);
    var lower = math.eval("2/1 m").toSI().toNumber();

    if (value === undefined) {
        throw "inThousandsMatrixArray: Could not evaluate input";
    }

    //check dimension
    var dim = math.matrix([1111, 1234, 3, 1200300]);
    if (!math.deepEqual(math.size(value), dim)) {
        throw "inThousandsMatrixArray: Input has dimension " + math.size(value) + " but expected " + dim;
    }

    var array = [];
    for (var i0 = 0; i0 < 1111; i0++) {
        array[i0] = [];
        for (var i1 = 0; i1 < 1234; i1++) {
            array[i0][i1] = [];
            for (var i2 = 0; i2 < 3; i2++) {
                array[i0][i1][i2] = [];
                for (var i3 = 0; i3 < 1200300; i3++) {
                    var e = value.get([i0, i1, i2, i3]);

                    //check unit
                    var expectedUnit = math.eval("m");
                    if (math.typeof(expectedUnit) !== math.typeof(e) || !expectedUnit.equalBase(e)) {
                        throw "inThousandsMatrixArray: Expected unit m";
                    }
                    var e_num = e.toSI().toNumber();
                    //check range
                    if (math.smaller(e_num, lower)) {
                        throw "inThousandsMatrixArray: Value " + e_num + " out of range";
                    }
                    array[i0][i1][i2][i3] = e_num;
                }
            }
        }
    }
    Module.setInThousandsMatrixArray(array);
}

