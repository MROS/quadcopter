console.log("Hello?");

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
            console.log(typeof(chCode));
            console.log("down");
            console.log(chCode);
            var dir = keyMapDir[chCode];
            if (dir != null) {
                changeColor[dir]();
            }
}
document.onkeyup = function(event){
            var chCode = event.keyCode;
            console.log(typeof(chCode));
            console.log("up");
            console.log(chCode);
            var dir = keyMapDir[chCode];
            if (dir != null) {
                changeBackColor[dir]();
            }
}