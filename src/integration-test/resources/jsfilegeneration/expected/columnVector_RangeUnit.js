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

function getOutRangeUnit() {
    return math.format(Module.getOutRangeUnit(), {notation: 'fixed'}).concat(" m/s");
}

function setInRangeUnit(param) {
    var value = math.eval(param);

    if (value === undefined) {
        throw "Could not evaluate input for param";
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
            var expectedUnit = math.eval("m/s");
            if (math.typeof(expectedUnit) !== math.typeof(e) || !expectedUnit.equalBase(e)) {
                throw "Expected unit m/s";
            }
            //check range
            if (math.smaller(e, math.eval("-10/3 m/s"))) {
                throw "Value " + e + " out of range";
            }
            if (math.larger(e, math.eval("10/1 km/h"))) {
                throw "Value " + e + " out of range";
            }
            array[i0][i1] = e.toSI().toNumber();
        }
    }
    Module.setInRangeUnit(array);
}

