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
    return math.format(Module.getOutRangeNoUnit(), {notation: 'fixed'})
        ;
}

function setInRangeNoUnit(_inRangeNoUnit) {
    var value = math.eval(_inRangeNoUnit);
    var lower = math.eval("-10/1").toSI().toNumber();
    var upper = math.eval("10/1").toSI().toNumber();

    if (value === undefined) {
        throw "Could not evaluate input for _inRangeNoUnit";
    }

    //check dimension
    var dim = math.matrix([1]);
    if (!math.deepEqual(value.size(), dim)) {
        throw "Input has dimension " + value.size() + " but expected " + dim;
    }

    var array = [];
    for (var i0 = 0; i0 < 1; i0++) {

        var e = value.get([i0]);

        //check unit
        var e_num = e.toSI().toNumber();
        //check range
        if (math.smaller(e_num, lower)) {
            throw "Value " + e_num + " out of range";
        }
        if (math.larger(e_num, upper)) {
            throw "Value " + e_num + " out of range";
        }
        array[i0] = e_num;
    }
    Module.setInRangeNoUnit(array);
}
