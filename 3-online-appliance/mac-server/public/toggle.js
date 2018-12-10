var buttonState = false;
var button = document.getElementById('button');
var state = document.getElementById('state');
var submit = document.getElementById('submit');
var url = 'http://smartsystems.ddns.net:1111/';

// lights on/off? --- buttonState --- textToDisplay
// on             --- true        --- OFF
// off            --- false       --- ON

button.addEventListener('click', (e) => {
  if(buttonState) {
    buttonState = false;
    button.innerHTML = "ON";
  } else {
    buttonState = true;
    button.innerHTML = "OFF";
  }
  var xhr = new XMLHttpRequest();
  xhr.open("POST", url, true);
  xhr.setRequestHeader('Content-Type', 'application/json');
  xhr.send(JSON.stringify({
      buttonState: buttonState
  }));
});

// submit.addEventListener('click', (e) => {
//   e.preventDefault();
// })


// state of brightness --- JSON
// BRIGHT              --- 1
// DARK                --- 0

setInterval( function() {
  var stateXhr = new XMLHttpRequest();
  stateXhr.open("GET", url+'adc', true);
  stateXhr.onload = function () {
    var resp = JSON.parse(stateXhr.response).adcState;
    // console.log(resp);
    if(resp == '1') {
      state.innerHTML = "BRIGHT";
      state.style.backgroundColor = "#FFFF00";
    } else {
      state.innerHTML = "DARK";
      state.style.backgroundColor = "#D3D3D3";
    }
  }
  stateXhr.send();
}, 500);
