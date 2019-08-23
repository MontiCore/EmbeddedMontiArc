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

function getAcceleration() {
    return math.format(Module.getAcceleration(), {notation: 'fixed'}).concat(" m/s^2");
}

function setPower(_power) {
    var value = math.eval(_power);
    var lower = math.eval("0/1 kg*m^2/s^3").toSI().toNumber();

    if (value === undefined) {
        throw "power: Could not evaluate input";
    }

    //check type
    if (math.typeof(value) !== "Unit") {
        throw "power: Expected type Unit";
    }

    //check unit
    var expectedUnit = math.eval("kg*m^2/s^3");
    if (math.typeof(expectedUnit) !== math.typeof(value) || !expectedUnit.equalBase(value)) {
        throw "power: Expected unit kg*m^2/s^3";
    }
    var value_num = value.toSI().toNumber();
    //check range
    if (math.smaller(value_num, lower)) {
        throw "power: Value " + value_num + " out of range";
    }
    Module.setPower(value_num);
}
