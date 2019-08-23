/* (c) https://github.com/MontiCore/monticore */
var Module = {
  'print': function (text) { console.log('stdout: ' + text) },
  'printErr': function (text) { console.log('stderr: ' + text) },
  onRuntimeInitialized: function () { Module.init(); }
};

function init() {
  Module.init();
}

function execute() {
  Module.execute();
}

  function getMarioDirection() {
    return math.format(Module.getMarioDirection(), {notation: 'fixed'})
  ;
  }
  function getMarioJump() {
    return math.format(Module.getMarioJump(), {notation: 'fixed'})
  ;
  }
  function getMarioDown() {
    return math.format(Module.getMarioDown(), {notation: 'fixed'})
  ;
  }
  function getMarioShoot() {
    return math.format(Module.getMarioShoot(), {notation: 'fixed'})
  ;
  }

  function setMarioPosition(_marioPosition) {
  var value = math.eval(_marioPosition);

  if (value === undefined) {
    throw "marioPosition: Could not evaluate input";
  }

    //check dimension
    var dim = math.matrix([1, 2]);
    if (!math.deepEqual(math.size(value), dim)) {
      throw "marioPosition: Input has dimension " + math.size(value) + " but expected " + dim;
    }

    var array = [];
  for (var i0 = 0; i0 < 1; i0++) {
    array  [i0]
 = [];
  for (var i1 = 0; i1 < 2; i1++) {
      var e = value.get([i0,i1]);

        var e_num = e;
      array  [i0][i1]
 = e_num;
  }
  }
    Module.setMarioPosition(array);
}
  function setMarioVelocity(_marioVelocity) {
  var value = math.eval(_marioVelocity);

  if (value === undefined) {
    throw "marioVelocity: Could not evaluate input";
  }

    //check dimension
    var dim = math.matrix([1, 2]);
    if (!math.deepEqual(math.size(value), dim)) {
      throw "marioVelocity: Input has dimension " + math.size(value) + " but expected " + dim;
    }

    var array = [];
  for (var i0 = 0; i0 < 1; i0++) {
    array  [i0]
 = [];
  for (var i1 = 0; i1 < 2; i1++) {
      var e = value.get([i0,i1]);

        var e_num = e;
      array  [i0][i1]
 = e_num;
  }
  }
    Module.setMarioVelocity(array);
}
  function setMarioHeight(_marioHeight) {
  var value = math.eval(_marioHeight);

  if (value === undefined) {
    throw "marioHeight: Could not evaluate input";
  }

  //check type
  if (math.typeof(value) !== "number") {
    throw "marioHeight: Expected type number";
  }
      var value_num = value;
    Module.setMarioHeight(value_num);
}
  function setNextEnemyPositions(_nextEnemyPositions) {
  var value = math.eval(_nextEnemyPositions);

  if (value === undefined) {
    throw "nextEnemyPositions: Could not evaluate input";
  }

    //check dimension
    var dim = math.matrix([5, 2]);
    if (!math.deepEqual(math.size(value), dim)) {
      throw "nextEnemyPositions: Input has dimension " + math.size(value) + " but expected " + dim;
    }

    var array = [];
  for (var i0 = 0; i0 < 5; i0++) {
    array  [i0]
 = [];
  for (var i1 = 0; i1 < 2; i1++) {
      var e = value.get([i0,i1]);

        var e_num = e;
      array  [i0][i1]
 = e_num;
  }
  }
    Module.setNextEnemyPositions(array);
}
  function setNextObstaclePositions(_nextObstaclePositions) {
  var value = math.eval(_nextObstaclePositions);

  if (value === undefined) {
    throw "nextObstaclePositions: Could not evaluate input";
  }

    //check dimension
    var dim = math.matrix([5, 2]);
    if (!math.deepEqual(math.size(value), dim)) {
      throw "nextObstaclePositions: Input has dimension " + math.size(value) + " but expected " + dim;
    }

    var array = [];
  for (var i0 = 0; i0 < 5; i0++) {
    array  [i0]
 = [];
  for (var i1 = 0; i1 < 2; i1++) {
      var e = value.get([i0,i1]);

        var e_num = e;
      array  [i0][i1]
 = e_num;
  }
  }
    Module.setNextObstaclePositions(array);
}
  function setNextHole(_nextHole) {
  var value = math.eval(_nextHole);

  if (value === undefined) {
    throw "nextHole: Could not evaluate input";
  }

  //check type
  if (math.typeof(value) !== "number") {
    throw "nextHole: Expected type number";
  }
      var value_num = value;
    Module.setNextHole(value_num);
}
  function setNextLootCrates(_nextLootCrates) {
  var value = math.eval(_nextLootCrates);

  if (value === undefined) {
    throw "nextLootCrates: Could not evaluate input";
  }

    //check dimension
    var dim = math.matrix([5, 2]);
    if (!math.deepEqual(math.size(value), dim)) {
      throw "nextLootCrates: Input has dimension " + math.size(value) + " but expected " + dim;
    }

    var array = [];
  for (var i0 = 0; i0 < 5; i0++) {
    array  [i0]
 = [];
  for (var i1 = 0; i1 < 2; i1++) {
      var e = value.get([i0,i1]);

        var e_num = e;
      array  [i0][i1]
 = e_num;
  }
  }
    Module.setNextLootCrates(array);
}
  function setTickSize(_tickSize) {
  var value = math.eval(_tickSize);

  if (value === undefined) {
    throw "tickSize: Could not evaluate input";
  }

  //check type
  if (math.typeof(value) !== "number") {
    throw "tickSize: Expected type number";
  }
      var value_num = value;
    Module.setTickSize(value_num);
}
  function setMarioResting(_marioResting) {
  var value = math.eval(_marioResting);

  if (value === undefined) {
    throw "marioResting: Could not evaluate input";
  }

  //check type
  if (math.typeof(value) !== "number") {
    throw "marioResting: Expected type number";
  }
      var value_num = value;
    Module.setMarioResting(value_num);
}

