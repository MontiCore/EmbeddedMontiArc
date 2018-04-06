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

function setInRangeNoUnit(_inRangeNoUnit) {
    var value = math.eval(_inRangeNoUnit);
    var lower = math.eval("-10/1").toSI().toNumber();
    var upper = math.eval("10/1").toSI().toNumber();

    if (value === undefined) {
        throw "Could not evaluate input for _inRangeNoUnit";
    }

    //check unit
    var value_num = value.toSI().toNumber();
    //check range
    if (math.smaller(value_num, lower)) {
        throw "Value " + value_num + " out of range";
    }
    if (math.larger(value_num, upper)) {
        throw "Value " + value_num + " out of range";
    }
    Module.setInRangeNoUnit(value_num);
}
