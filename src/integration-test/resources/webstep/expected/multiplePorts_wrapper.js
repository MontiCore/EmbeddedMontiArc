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

function execute() {
    Module.execute();
}

function getV() {
    return math.format(Module.getV(), {notation: 'fixed'}).concat(" km/h");
}

function getK() {
    return math.format(Module.getK(), {notation: 'fixed'}).concat(" km");
}


function setM1(_m1) {
    var value = math.eval(_m1);
    var lower = math.eval("0/1 m").toSI().toNumber();
    var upper = math.eval("10/1 km").toSI().toNumber();

    if (value === undefined) {
        throw "Could not evaluate input for _m1";
    }

//check dimension
    var dim = math.matrix([2, 3]);
    if (!math.deepEqual(value.size(), dim)) {
        throw "Input has dimension " + value.size() + " but expected " + dim;
    }

    var array = [];
    for (var i0 = 0; i0 < 2; i0++) {
        array[i0] = [];
        for (var i1 = 0; i1 < 3; i1++) {

            var e = value.get([i0, i1]);

            //check unit
            var expectedUnit = math.eval("m");
            if (math.typeof(expectedUnit) !== math.typeof(e) || !expectedUnit.equalBase(e)) {
                throw "Expected unit m";
            }

            var e_num = e.toSI().toNumber();
            //check range
            if (math.smaller(e_num, lower)) {
                throw "Value " + e_num + " out of range";
            }
            if (math.larger(e_num, upper)) {
                throw "Value " + e_num + " out of range";
            }
            array[i0][i1] = e_num;
        }
    }
    Module.setM1(array);
}

function setM2(_m2) {
    var value = math.eval(_m2);
    var lower = math.eval("5/1 Hz").toSI().toNumber();
    var upper = math.eval("20/1 kHz").toSI().toNumber();

    if (value === undefined) {
        throw "Could not evaluate input for _m2";
    }

//check dimension
    var dim = math.matrix([3, 1]);
    if (!math.deepEqual(value.size(), dim)) {
        throw "Input has dimension " + value.size() + " but expected " + dim;
    }

    var array = [];
    for (var i0 = 0; i0 < 3; i0++) {
        array[i0] = [];
        for (var i1 = 0; i1 < 1; i1++) {

            var e = value.get([i0, i1]);

            //check unit
            var expectedUnit = math.eval("Hz");
            if (math.typeof(expectedUnit) !== math.typeof(e) || !expectedUnit.equalBase(e)) {
                throw "Expected unit Hz";
            }

            var e_num = e.toSI().toNumber();
            //check range
            if (math.smaller(e_num, lower)) {
                throw "Value " + e_num + " out of range";
            }
            if (math.larger(e_num, upper)) {
                throw "Value " + e_num + " out of range";
            }
            array[i0][i1] = e_num;
        }
    }
    Module.setM2(array);
}

function setC(_c) {
    var value = math.eval(_c);
    var lower = math.eval("-2/1 mm/h").toSI().toNumber();
    var upper = math.eval("2/1 km/s").toSI().toNumber();

    if (value === undefined) {
        throw "Could not evaluate input for _c";
    }

    //check unit
    var expectedUnit = math.eval("mm/h");
    if (math.typeof(expectedUnit) !== math.typeof(value) || !expectedUnit.equalBase(value)) {
        throw "Expected unit mm/h";
    }

    var value_num = value.toSI().toNumber();
    //check range
    if (math.smaller(value_num, lower)) {
        throw "Value " + value_num + " out of range";
    }
    if (math.larger(value_num, upper)) {
        throw "Value " + value_num + " out of range";
    }
    Module.setC(value_num);
}

