var express = require('express');
var bodyParser = require('body-parser');
var app = express();
var http = require('http');
var request = require('request');
var cheerio = require('cheerio');

var encoded;
var encodedReceived = false;

app.use(express.static('public'));
app.use(bodyParser.urlencoded({extended: false}));
app.use(bodyParser.json());

// Points to index.html to serve web page
app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});

app.post('/', (req, res) => {
  var dir = req.body.direction;
  console.log(dir);
  request.post('http://remotecontrol.ddns.net:80', {form: {key: dir}});
  res.end();
});

app.post('/car', (req, res) => {
  app.use(bodyParser.text({type: 'application/x-www-form-urlencoded'}));
  var body = req.body;
  encoded = Object.keys(body)[0];
  encodedReceived = true;
  console.log(encoded);
});

app.get('/encoded', (req, res) => {
  res.set('Content-Type', 'application/json');
  if(encodedReceived = true) {
    res.send({msg: encoded});
  }
});

// Listening on localhost:3000
app.listen(1111, function() {
  console.log('listening on *:1111');
});
