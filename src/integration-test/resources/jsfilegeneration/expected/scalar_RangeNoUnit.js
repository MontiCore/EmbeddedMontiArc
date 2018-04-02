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

    //check unit
    //check range
    if (math.smaller(value, math.eval("-10/1"))) {
        throw "Value " + value + " out of range";
    }
    if (math.larger(value, math.eval("10/1"))) {
        throw "Value " + value + " out of range";
    }
    Module.setInRangeNoUnit(value.toSI().toNumber());
}
