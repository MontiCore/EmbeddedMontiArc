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

function getAcceleration() {
    return math.format(Module.getAcceleration(), {notation: 'fixed'}).concat(" m/s^2");
}

function setPower(_power) {
    var value = math.eval(_power);

    if (value === undefined) {
        throw "Could not evaluate input for _power";
    }

    //check unit
    var expectedUnit = math.eval("kg*m^2/s^3");
    if (math.typeof(expectedUnit) !== math.typeof(value) || !expectedUnit.equalBase(value)) {
        throw "Expected unit kg*m^2/s^3";
    }
    var value_num = value.toSI().toNumber();
    //check range
    if (math.smaller(value_num, 0 / 1)) {
        throw "Value " + value_num + " out of range";
    }
    Module.setPower(value_num);
}
