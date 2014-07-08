var wSocket = new WebSocket("ws://localhost:8888/ws");

wSocket.onopen = function(){
	wSocket.send("who are you?");
}
wSocket.onmessage = function(event){
	console.log(event.data);
}



//document.onkeypress = function(event){
//            var chCode = event.charCode;
//                console.log("press");
//                console.log(chCode);
//            }

var changeColor = {'left': function(){document.getElementById("arrow-left").style.borderRight = "80px solid red";}
                ,'right': function(){document.getElementById("arrow-right").style.borderLeft = "80px solid red";}
                ,'front': function(){document.getElementById("arrow-up").style.borderBottom = "80px solid red";}
                ,'rear': function(){document.getElementById("arrow-down").style.borderTop = "80px solid red";}}
var changeBackColor = {'left': function(){document.getElementById("arrow-left").style.borderRight = "80px solid blue";}
                ,'right': function(){document.getElementById("arrow-right").style.borderLeft = "80px solid blue";}
                ,'front': function(){document.getElementById("arrow-up").style.borderBottom = "80px solid blue";}
                ,'rear': function(){document.getElementById("arrow-down").style.borderTop = "80px solid blue";}}

var keyMapDir = {
    37: 'left',
    38: 'front',
    39: 'right',
    40: 'rear'
};

document.onkeydown = function(event){
            var chCode = event.keyCode;
            var dir = keyMapDir[chCode];
            if (dir != null) {
                changeColor[dir]();
				//wSocket.send(dir + "_add")
            }
}
document.onkeyup = function(event){
            var chCode = event.keyCode;
            var dir = keyMapDir[chCode];
            if (dir != null) {
                changeBackColor[dir]();
            }
}
