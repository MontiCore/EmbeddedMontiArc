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
    return math.format(Module.getOutRangeUnit(), {notation: 'fixed'})
        .concat(" m/s");
}

function setInRangeUnit(param) {
    var value = math.eval(param);

    if (value === undefined) {
        throw "Could not evaluate input for param";
    }

    //check unit
    var expectedUnit = math.eval("m/s");
    if (math.typeof(expectedUnit) !== math.typeof(value) || !expectedUnit.equalBase(value)) {
        throw "Expected unit m/s";
    }
    //check range
    if (math.smaller(value, math.eval("-10/3 m/s"))) {
        throw "Value " + value + " out of range";
    }
    if (math.larger(value, math.eval("10/1 km/h"))) {
        throw "Value " + value + " out of range";
    }
    Module.setInRangeUnit(value.toSI().toNumber());
}

