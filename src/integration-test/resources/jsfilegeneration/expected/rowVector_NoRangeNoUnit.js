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

function getOutNoRangeNoUnit() {
    return math.format(Module.getOutNoRangeNoUnit(), {notation: 'fixed'});
}

function setInNoRangeNoUnit(_inNoRangeNoUnit) {
    var value = math.eval(_inNoRangeNoUnit);

    if (value === undefined) {
        throw "inNoRangeNoUnit: Could not evaluate input";
    }

    //check dimension
    var dim = math.matrix([1, 3]);
    if (!math.deepEqual(math.size(value), dim)) {
        throw "inNoRangeNoUnit: Input has dimension " + math.size(value) + " but expected " + dim;
    }

    var array = [];
    for (var i0 = 0; i0 < 1; i0++) {
        array[i0] = [];
        for (var i1 = 0; i1 < 3; i1++) {

            var e = value.get([i0, i1]);

            var e_num = e;
            array[i0][i1] = e_num;
        }
    }
    Module.setInNoRangeNoUnit(array);
}
