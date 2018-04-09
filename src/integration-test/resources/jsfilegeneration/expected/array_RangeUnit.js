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

function getOutRangeUnit() {
    return math.format(Module.getOutRangeUnit(), {notation: 'fixed'}).concat(" m/s");
}

function setInRangeUnit(_inRangeUnit) {
    var value = math.eval(_inRangeUnit);

    if (value === undefined) {
        throw "Could not evaluate input for _inRangeUnit";
    }

    //check dimension
    var dim = math.matrix([4]);
    if (!math.deepEqual(value.size(), dim)) {
        throw "Input has dimension " + value.size() + " but expected " + dim;
    }

    var array = [];
    for (var i0 = 0; i0 < 4; i0++) {
        var e = value.get([i0]);

        //check unit
        var expectedUnit = math.eval("m/s");
        if (math.typeof(expectedUnit) !== math.typeof(e) || !expectedUnit.equalBase(e)) {
            throw "Expected unit m/s";
        }
        var e_num = e.toSI().toNumber();
        //check range
        if (math.smaller(e_num, -10 / 3)) {
            throw "Value " + e_num + " out of range";
        }
        if (math.larger(e_num, 10 / 1)) {
            throw "Value " + e_num + " out of range";
        }
        array[i0] = e_num;
    }
    Module.setInRangeUnit(array);
}

