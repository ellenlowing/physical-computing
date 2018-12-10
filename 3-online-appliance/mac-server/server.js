var express = require('express');
var bodyParser = require('body-parser');
var app = express();
var http = require('http');

var buttonState = false;
var adcState = "0";
var schedule = {
  hour: 0,
  minute: 0
}

// serve files from the public directory
app.use(express.static('public'));

// configure body-parser for express
app.use(bodyParser.urlencoded({extended: false}));
app.use(bodyParser.json());

// serve the homepage
app.get('/', (req, res) => {
  res.sendFile(__dirname + '/index.html');
});

// handle GET request from python client
app.get('/ctrl', (req, res) => {
  if(buttonState) res.end("1")
  else res.end("0");
});

app.get('/adc', (req, res) => {
  res.set('Content-Type', 'application/json');
  res.send( {adcState: adcState} );
});

app.get('/scheduler', (req, res) => {
  res.set('Content-Type', 'application/json');
  res.send( schedule );
});

// handle POST request
app.post('/', (req, res) => {
  response = {
    buttonState: req.body.buttonState
  };
  buttonState = response.buttonState;
  res.end();
});

app.post('/scheduler', (req, res) => {
  schedule.hour = req.body.hour;
  schedule.minute = req.body.minute;
  res.end();
});

// handle PUT request from python client
app.post('/adc', (req, res) => {
  response = {
    adc_reading: req.body.adc_reading
  };
  // if(response.adc_reading == "1") adcState = true;
  // else adcState = false;
  adcState = response.adc_reading;
  // console.log("ADC:  " + response.adc_reading);
  // res.send(response.adc_reading);
  res.end();
});



// start the express web server listening on 1111
app.listen(1111, () => {
  console.log('listening on 1111');
});
