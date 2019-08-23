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

function getOutRangeNoUnit() {
    return math.format(Module.getOutRangeNoUnit(), {notation: 'fixed'});
}

function setInRangeNoUnit(_inRangeNoUnit) {
    var value = math.eval(_inRangeNoUnit);
    var lower = -10 / 1;
    var upper = 10 / 1;

    if (value === undefined) {
        throw "inRangeNoUnit: Could not evaluate input";
    }

    //check type
    if (math.typeof(value) !== "number") {
        throw "inRangeNoUnit: Expected type number";
    }

    var value_num = value;
    //check range
    if (math.smaller(value_num, lower)) {
        throw "inRangeNoUnit: Value " + value_num + " out of range";
    }
    if (math.larger(value_num, upper)) {
        throw "inRangeNoUnit: Value " + value_num + " out of range";
    }
    Module.setInRangeNoUnit(value_num);
}
