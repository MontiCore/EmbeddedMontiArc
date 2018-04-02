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

function getOutRangeNoUnit() {
    return math.format(Module.getOutRangeNoUnit(), {notation: 'fixed'});
}

function setInRangeNoUnit(param) {
    var value = math.eval(param);

    if (value === undefined) {
        throw "Could not evaluate input for param";
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
            //check range
            if (math.smaller(e, math.eval("-10/1"))) {
                throw "Value " + e + " out of range";
            }
            if (math.larger(e, math.eval("10/1"))) {
                throw "Value " + e + " out of range";
            }
            array[i0][i1] = e.toSI().toNumber();
        }
    }
    Module.setInRangeNoUnit(array);
}

