/* (c) https://github.com/MontiCore/monticore */
    function getPng() {
        var xmlhttp = new XMLHttpRequest();
        xmlhttp.onreadystatechange = function() {
            if (xmlhttp.readyState == XMLHttpRequest.DONE) {   // XMLHttpRequest.DONE == 4
               if (xmlhttp.status == 200) {
                   var plantUMLString = xmlhttp.responseText;
                   compress(plantUMLString);
               }
               else if (xmlhttp.status == 400) {
                   alert('There was an error 400');
               }
               else {
                   alert('something else other than 200 was returned');
               }
            }
        };

        xmlhttp.open("GET", "data/plantUML.txt", true);
        xmlhttp.send();

    }

    // code below from http://plantuml.com/code-javascript-asynchronous
    function encode64(data) {
    	r = "";
    	for (i=0; i<data.length; i+=3) {
     		if (i+2==data.length) {
    			r +=append3bytes(data.charCodeAt(i), data.charCodeAt(i+1), 0);
    		} else if (i+1==data.length) {
    			r += append3bytes(data.charCodeAt(i), 0, 0);
    		} else {
    			r += append3bytes(data.charCodeAt(i), data.charCodeAt(i+1),
    				data.charCodeAt(i+2));
    		}
    	}
    	return r;
    }

    function append3bytes(b1, b2, b3) {
    	c1 = b1 >> 2;
    	c2 = ((b1 & 0x3) << 4) | (b2 >> 4);
    	c3 = ((b2 & 0xF) << 2) | (b3 >> 6);
    	c4 = b3 & 0x3F;
    	r = "";
    	r += encode6bit(c1 & 0x3F);
    	r += encode6bit(c2 & 0x3F);
    	r += encode6bit(c3 & 0x3F);
    	r += encode6bit(c4 & 0x3F);
    	return r;
    }

    function encode6bit(b) {
    	if (b < 10) {
     		return String.fromCharCode(48 + b);
    	}
    	b -= 10;
    	if (b < 26) {
     		return String.fromCharCode(65 + b);
    	}
    	b -= 26;
    	if (b < 26) {
     		return String.fromCharCode(97 + b);
    	}
    	b -= 26;
    	if (b == 0) {
     		return '-';
    	}
    	if (b == 1) {
     		return '_';
    	}
    	return '?';
    }

    var deflater = window.SharedWorker && new SharedWorker('js/rawdeflate.js');
    if (deflater) {
    	deflater.port.addEventListener('message', done_deflating, false);
    	deflater.port.start();
    } else if (window.Worker) {
    	deflater = new Worker('js/rawdeflate.js');
    	deflater.onmessage = done_deflating;
    }

    function done_deflating(e) {
    	 document.getElementById("cd-output").src = "https://www.plantuml.com/plantuml/svg/"+encode64(e.data);
    }

    function compress(s) {
    	//UTF8
    	s = unescape(encodeURIComponent(s));

    	if (deflater) {
    		if (deflater.port && deflater.port.postMessage) {
    			deflater.port.postMessage(s);
    		} else {
    			deflater.postMessage(s);
    		}
      	} else {
     		setTimeout(function() {
    	  		done_deflating({ data: deflate(s) });
    		}, 100);
      	}
    }
