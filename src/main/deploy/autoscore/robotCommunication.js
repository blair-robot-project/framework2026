//Instantiate the new client
// using `window.location.hostname` causes the client to open a 
// NT connection on the same machine as is serving the website.
// It could be hardcoded to point at a roboRIO if needed.
const nt4Client = new NT4_Client(
  window.location.hostname, 
  topicAnnounceHandler,
  topicUnannounceHandler,
  valueUpdateHandler,
  parameterChange,
  onConnect,
  onDisconnect
);
// Allocate a variable to hold the subscription to all topics
var subscription = null;

/**
* Topic Announce Handler
* The NT4_Client will call this function whenever the server announces a new topic.
* It's the user's job to react to the new topic in some useful way.
* @param {NT4_Topic} newTopic The new topic the server just announced.
*/
function topicAnnounceHandler( newTopic ) {
  console.log("topic announced: " + newTopic.name);

}

/**
* Topic UnAnnounce Handler
* The NT4_Client will call this function whenever the server un-announces a topic.
* It's the user's job to react to the un-anncouncement in some useful way.
* @param {NT4_Topic} removedTopic The topic the server just un-announced.
*/
function topicUnannounceHandler( removedTopic ) {
}

/**
* Value Update Handler
* The NT4_Client will call this function whenever the server sends a value update
* for a topic.
* @param {NT4_Topic} topic The topic with a value update
* @param {double} timestamp_us The server time of the value update occurring
* @param {*} value the new value for the topic
*/
function valueUpdateHandler( topic, timestamp_us, value ) {
  console.log("value updated: " + topic.name);
  if(topic.name == "/webcom/Alliance") {
    setAlliance(value);
  } else if(topic.name == "/webcom/isDone" && value===true) {
    document.getElementById("manualText").innerText = `Scoring on Area ${numberToLetter[11-(reefArea+4)%12]} and Level ${coralLevel}`;
    showContainer("manualContainer");
    ["pivotForward", "pivotBack", "elevatorUp", "elevatorDown", "wristForward", "wristBack"].forEach(
      id => {
        const elt = document.getElementById(id);
        const eltinterval = setInterval(() => {
          if(window.getComputedStyle(elt).backgroundColor == "rgb(200, 0, 0)") {
            setCommand(id);
          }
        }, 0.02);
        intervalList.push(eltinterval);
      }
    );
  }
}

/**
* On Connection Handler
* The NT4_Client will call this after every time it successfully connects to an NT4 server.
*/
function onConnect() {
  console.log("connected to robot");
  subscription = nt4Client.subscribePeriodic(["/webcom"], 0.1);
  connection();
}

/**
* On Disconnection Handler
* The NT4_Client will call this after every time it disconnects to an NT4 server.
*/
function onDisconnect() {
  console.log("disconnected from robot");
  //For this example, we simply mark the status as disconnected.
  noConnection();
  //Since we've disconnected from the server, the connection is no longer valid.
  subscription = null;
}

function parameterChange(changeObject) {
  //gives an object with the change
}

const setAlliance = (alliance) => {
  if(alliance == "Red") {
      document.body.style.background = "radial-gradient(circle at 50% 50%, pink, rgb(114, 114, 138))";
  } else if(alliance == "Blue") {
      document.body.style.background = "radial-gradient(circle at 50% 50%, cornflowerblue, rgb(114, 114, 138))";
  }
}

const commandPath = "/webcom/Command";
const isDonePath = "/webcom/isDone";

const commandPublishTopic = nt4Client.publishNewTopic(commandPath, NT4_TYPESTR.STR);

setCommand = (command) => {
  nt4Client.addSample(commandPath, nt4Client.getServerTime_us(), command);
}


let coralLevel = -1;
let reefArea = -1;
let coralSelected = false;
let areaSelected = false;
let intervalList = [];

const resetReef = () => {
  coralSelected = false;
  areaSelected = false;
  coralLevel = -1;
  document.getElementById("confirmReefButton").innerText = `Choose Robot Alignment`;
  document.getElementById("areaSelectionText").innerText = "Hover to the area you want to go to.";
  document.getElementById("coral").src = `coralLevelImages/coralNone.png`;
  document.getElementById("areaText").innerText = "Reef Area: None";
  document.getElementById("coralText").innerText = "Coral Level: None";
  document.getElementById("locationSelect").src = "locationSelectorImages/locationSelectorNone.png";
  intervalList.forEach(ivl => clearInterval(ivl));
  intervalList.length = 0;
}

document.getElementById("confirmReefButton").onclick = async () => {
  if(coralSelected && areaSelected) {
    scoreReef(reefArea, coralLevel);
    document.getElementById("confirmReefButton").innerText = "Scoring...";
  }
}


const scoreNet = async (isOnRedAllianceSide) => {
  if(isOnRedAllianceSide) {
    setCommand("netRed");
  } else {
    setCommand("netBlue");
  }
}

const scoreProcessor = async () => {
  console.log("scoring processor");
  setCommand("processor");
}

const intakeCoral = async (isAtTopSide) => {
  if(isAtTopSide) {
    setCommand("intakeCoralTop");
  } else {
    setCommand("intakeCoralBottom");
  }
}

const scoreReef = async (location, level) => {
  console.log(`Scoring on level: ${level} and location: ${location}`)
  setCommand(`l${level} location${location > 3 ? location-3 : location+9}`);
}

const scoringContainer = document.querySelector(".scoringContainer");
const messageContainer = document.querySelector(".messageContainer");
const msgDisplay = document.getElementById("messageDisplay");

const processorContainer = document.querySelector(".processorContainer");
const netContainer = document.querySelector(".netContainer");
const coralIntakeContainer = document.querySelector(".coralIntakeContainer");

const noConnection = () => {
  scoringContainer.style.display = "none";
  messageContainer.style.display = "flex";
  msgDisplay.innerText = "NO ROBOT CONNECTION";
  document.body.style.background = "rgb(114, 114, 138)";
}

const connection = () => {
  messageContainer.style.display = "none";
  scoringContainer.style.display = "flex";
  msgDisplay.innerText = "";
  resetReef();
  showContainer("reefContainer");
}

document.getElementById("cancel").onclick = () => {
  setCommand("cancel");
  resetReef();
  showContainer("reefContainer");
}

document.getElementById("score").onclick = () => {
  setCommand("score");
  resetReef();
  showContainer("reefContainer");
}

let currentContainer = "reefContainer";

const showContainer = (container) => {
  document.querySelector(`.${currentContainer}`).style.display = "none";	
  currentContainer = container;
  document.querySelector(`.${currentContainer}`).style.display = "flex";
}

noConnection();

const numberToLetter = [
  'A',	
  'B',	
  'C',	
  'D',	
  'E',	
  'F',	
  'G',	
  'H',	
  'I',	
  'J',	
  'K',
  'L'
];