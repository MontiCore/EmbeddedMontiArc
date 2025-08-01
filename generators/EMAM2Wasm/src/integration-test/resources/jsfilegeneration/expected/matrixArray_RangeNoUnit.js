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

    //check dimension
    var dim = math.matrix([4, 2, 3]);
    if (!math.deepEqual(math.size(value), dim)) {
        throw "inRangeNoUnit: Input has dimension " + math.size(value) + " but expected " + dim;
    }

    var array = [];
    for (var i0 = 0; i0 < 4; i0++) {
        array[i0] = [];
        for (var i1 = 0; i1 < 2; i1++) {
            array[i0][i1] = [];
            for (var i2 = 0; i2 < 3; i2++) {

                var e = value.get([i0, i1, i2]);

                var e_num = e;
                //check range
                if (math.smaller(e_num, lower)) {
                    throw "inRangeNoUnit: Value " + e_num + " out of range";
                }
                if (math.larger(e_num, upper)) {
                    throw "inRangeNoUnit: Value " + e_num + " out of range";
                }
                array[i0][i1][i2] = e_num;
            }
        }
    }
    Module.setInRangeNoUnit(array);
}
