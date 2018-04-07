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

    if (value === undefined) {
        throw "Could not evaluate input for _inRangeNoUnit";
    }

    var value_num = value;
    //check range
    if (math.smaller(value_num, -10 / 1)) {
        throw "Value " + value_num + " out of range";
    }
    if (math.larger(value_num, 10 / 1)) {
        throw "Value " + value_num + " out of range";
    }
    Module.setInRangeNoUnit(value_num);
}
