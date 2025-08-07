/* (c) https://github.com/MontiCore/monticore */
var KEY = {
	UP: 38,
	DOWN: 40,
	LEFT: 37,
	RIGHT: 39,
	SHOOT: 17,
}

var supermario_simulator_ticksize = 50;

function startSimulation() {
    //PACMAN.startNewGame();
    try {
        window.setInterval(doSimulationStep, supermario_simulator_ticksize); 
    } catch (err) {
        if (err.message === undefined) {
            console.log(err)
        }
        else {
            console.log(err.message);
        }
    }
}

function pressKey(dir) {
    var keyCode = dir;


    // document.dispatchEvent(new KeyboardEvent('keydown',{'keyCode':keyCode}));
    var keyboardEvent = new KeyboardEvent('keydown', {bubbles:true});
    Object.defineProperty(keyboardEvent, 'keyCode', {get:function(){return keyCode;}});
    keyboardEvent.keyCode = [keyCode];
    document.body.dispatchEvent(keyboardEvent);
}

function releaseKey(dir) {
    var keyCode = dir;


    // document.dispatchEvent(new KeyboardEvent('keydown',{'keyCode':keyCode}));
    var keyboardEvent = new KeyboardEvent('keyup', {bubbles:true});
    Object.defineProperty(keyboardEvent, 'keyCode', {get:function(){return keyCode;}});
    keyboardEvent.keyCode = [keyCode];
    document.body.dispatchEvent(keyboardEvent);
}

function extractNextObstaclePositions(_solid){
	//Floor is at Y = 0, coordinate system goes up in y axis, right in x axis.
	
	//use player ypos, if he is "resting"
	var positions = [[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1]];
	var currentPos = 0;
	
	var playerY = -1;
	if(typeof FSM.player.resting != 'undefined'){
		playerY = FSM.player.resting.y;
	}
	
	for(entryNr in _solid){
		var entry = _solid[entryNr];
		//var isObstacle = false;
		
		if(entry.y > playerY && entry.y-entry.height <= playerY+10 && ((entry.left - FSM.player.left) > -35)){
			//isObstacle = true;
		
			//positions[currentPos][0] = entry.x;
			//positions[currentPos][1] = entry.y - playerY;
			
			positions[currentPos][0] = entry.left - FSM.player.left;
			positions[currentPos][1] = (entry.top - FSM.player.top)* -1;
			currentPos++;
		}
		
		
		if(currentPos >= 5){
			break;
		}
	}
	
	return positions;
}

function extractNextEnemyPositions(){
	var positions = [[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1]];
	var currentPos = 0;
	var characters = FSM.GroupHolder.groups.Character;
	
	for(entryNr in characters){
		var entry = characters[entryNr]; 
		//console.log(entry)
		if(entry.thing == "Goomba" || entry.thing == "Koopa"){
			//positions[currentPos][0] = entry.x;
			//positions[currentPos][1] = entry.y-entry.height;
			
			positions[currentPos][0] = entry.left-FSM.player.left;
			positions[currentPos][1] = entry.top-FSM.player.top;
			currentPos++;
		}
		
		if(currentPos >= 5){
			break;
		}
		
	}
	
	return positions;
	
}

function extractNextHolePosition(_solid){
	var screenWidth = FSM.canvas.width;
	var ret = -1;
	
	for(entryNr in _solid){
		var entry = _solid[entryNr];
		if(entry.thing =="Floor"){
			//var holePos = (entry.left + entry.width) - FSM.player.left;
			holePos = (entry.right) - FSM.player.left;
			if(holePos <= screenWidth){
				if((ret == -1  ||  holePos < ret) && holePos > 0){
					ret = holePos;
				}
			}
		}
	}
	
	return ret;
	
}


function sendPlayerData(){

	var player = FSM.player;
	//var marioPos = math.matrix([player.left, player.top]);
	//var marioVel = math.matrix([player.xvel, player.yvel]);
	//var marioHeight = math.matrix([player.height]);
	
	var marioPos = "[["+player.left+","+player.top+"]]";
	var marioVel = "[["+player.xvel+","+player.yvel+"]]";
	var marioHeight = String(player.height);
	var marioResting = 0;
	
	console.log("Player: "+marioPos);
	
	if(typeof FSM.player.resting != 'undefined'){
		console.log(FSM.player.resting);
		marioResting = 1;
	}
	
	setMarioResting(marioResting);
	setMarioPosition(marioPos);
	setMarioVelocity(marioVel);
	setMarioHeight(marioHeight);
	
}

function makeJSStringFromArray(_array){
	var out = "";
	
	if(Array.isArray(_array)){
		//console.log(_array)
		
		out = "[";
		for (entry in _array){
			//console.log(_array[entry])
			out = out + makeJSStringFromArray(_array[entry])+",";
		}
		out = out.slice(0, -1);
		out += "]";
	}else{
		out = String(_array);
	}
	
	return out;
}

function sendEnvironmentData(){
	var solid = FSM.GroupHolder.groups.Solid;
	
	
	var nextObstacles = extractNextObstaclePositions(solid);
	var nextEnemies = extractNextEnemyPositions();
	var nextHole = extractNextHolePosition(solid);
	
	console.log(makeJSStringFromArray(nextObstacles));
	
	var obstArray = makeJSStringFromArray(nextObstacles);
	var enemArray = makeJSStringFromArray(nextEnemies);
	
	
	
	console.log("Obstacles: "+obstArray);
	console.log("Enemies: " +enemArray);
	console.log("Hole: "+nextHole);
	
	setTickSize(1000 / supermario_simulator_ticksize );
	
	setNextObstaclePositions(obstArray);
	setNextEnemyPositions(enemArray);
	setNextHole(nextHole);
	
	setNextLootCrates("[[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1]]");
	
	

}

function getCommands(){
	var dir = getMarioDirection();
	var jump = getMarioJump();
	var shoot = getMarioShoot();
	var down = getMarioDown();
	
	if(dir==1){
		releaseKey(KEY.LEFT);
		pressKey(KEY.RIGHT);
	}else{
		releaseKey(KEY.RIGHT);
		if(dir==-1)
			pressKey(KEY.LEFT);
		else{
			releaseKey(KEY.LEFT);
			releaseKey(KEY.RIGHT);
		}
	}
		
	if(jump==1)
		pressKey(KEY.UP);
	else
		releaseKey(KEY.UP);
	
	if(shoot==1)
		pressKey(KEY.CONTROL);
	else
		releaseKey(KEY.CONTROL);
	
	if(down==1)
		pressKey(KEY.DOWN);
	else
		releaseKey(KEY.DOWN);
	
	//Debug
	console.log("Jump: "+jump);
	console.log("Shoot: "+shoot);
	console.log("Direction: "+dir);
	
}

function doSimulationStep(){
	sendPlayerData();
	sendEnvironmentData();
	execute();
	getCommands();
}


