var KEY = {
	UP: 38,
	DOWN: 40,
	LEFT: 37,
	RIGHT: 39,
	SHOOT: 17,
}



function startSimulation() {
    //PACMAN.startNewGame();
    try {
        window.setInterval(doSimulationStep, 500);
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

function extractNextObstaclePositions(_solid){
	//Floor is at Y = 0, coordinate system goes up in y axis, right in x axis.
	
	//use player ypos, if he is "resting"
	var positions = [[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1]];
	var currentPos = 0;
	
	var playerY = -1;
	if(FSM.Player.resting != null){
		playerY = FSM.Player.resting[y];
	}
	
	for(entry in _solid){
		//var isObstacle = false;
		
		if(entry[y] > playerY && entry[y]-entry[height] <= playerY){
			//isObstacle = true;
		
		
			positions[currentPos][0] = entry[x];
			positions[currentPos][1] = entry[y] - playerY;
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
	
	for(entry in characters){
		if(entry[thing] == "Goomba" || entry[thing] == "Koopa"){
			positions[currentPos][0] = entry[x];
			positions[currentPos][1] = entry[y]-entry[height];
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
	
	for(entry in _solid){
	
		if(entry[thing] =="Floor"){
			var holePos = entry[x] + entry[width];
			if(holePos <= screenWidth){
				ret = holePos;
			}
		}
	}
	
	return ret;
	
}

function sendPlayerData(){

	var player = FSM.player;
	var marioPos = math.matrix([player.left, player.top]);
	var marioVel = math.matrix([player.xvel, player.yvel]);
	var marioHeight = math.matrix([player.height]);
	
	console.log("Player: "+player);
	
	setMarioPosition(marioPos);
	setMarioVelocity(marioVel);
	setMarioHeight(marioHeight);
	
	
}

function sendEnvironmentData(){
	var solid = FSM.GroupHolder.group.Solid;
	
	
	var nextObstacles = math.matrix(extractNextObstaclePositions(solid));
	var nextEnemies = math.matrix(extractNextObstaclePositions(solid));
	var nextHole = math.matrix(extractNextHolePosition());
	
	
	console.log("Obstacles: "+nextObstacles);
	console.log("Enemies: " +nextEnemies);
	console.log("Hole: "+nextHole);
	
	setNextObstaclePositions(nextObstaclePositions);
	setNextEnemyPositions(nextEnemyPositions);
	setNextHole(nextHole);
	
	

}

function getCommands(){
	var dir = getMarioDirection();
	var jump = getMarioJump();
	var shoot = getMarioShoot();
	
	if(dir==1)
		pressKey(KEY.RIGHT);
	else
		if(dir==-1)
			pressKey(KEY.RIGHT);
		
	if(jump==true)
		pressKey(KEY.UP);
	
	if(shoot==true)
		pressKey(KEY.CONTROL);
	
	
	
}

function doSimulationStep(){
	sendPlayerData();
	sendEnvironmentData();
	
	getCommands();
}


