const locationImg = document.getElementById("locationSelect");
const confirmReefButton = document.getElementById("confirmReefButton");

let inPicker = false;
locationImg.onmouseenter = () => inPicker = true;
locationImg.onmouseleave = () => {
    if(!areaSelected) {
        inPicker = false;
        locationImg.src = "locationSelectorImages/locationSelectorNone.png";
    }
}

document.addEventListener("mousemove", (event) => {
    if(inPicker && !areaSelected) {
        let imgPosition = locationImg.getBoundingClientRect();
        let imageCenterX = imgPosition.left + (imgPosition.right-imgPosition.left) / 2;
        let imageCenterY = imgPosition.top + (imgPosition.bottom-imgPosition.top) / 2;
        
        let x = event.clientX-imageCenterX;
        let y = imageCenterY-event.clientY;

        let angleRadians = Math.atan2(x, y);
        let angleDegrees = angleRadians * (180 / Math.PI);
        if(angleRadians < 0) {
            angleDegrees = 360 + angleRadians * (180 / Math.PI);
        }
        reefArea = Math.floor(((angleDegrees-120) % 360)/30) + 5;
        if(reefArea == -1) {
            locationImg.src = `locationSelectorImages/locationSelectorNone.png`;
        } else {
            locationImg.src = `locationSelectorImages/locationSelector${reefArea < 5 ? reefArea+8 : reefArea-4}.png`;
        }
    }
});

let areaText = document.getElementById("areaText");
let areaSelectionText = document.getElementById("areaSelectionText");

locationImg.onclick = () => {
    areaSelected = !areaSelected;
    if(areaSelected) {
        areaText.innerText = `Reef Area: ${numberToLetter[11-(reefArea+4)%12]}`;
        areaSelectionText.innerText = "Click again to reselect";
    } else {
        confirmReefButton.innerText = `Choose Robot Alignment`;
        areaText.innerText = `Reef Area: None`;
        areaSelectionText.innerText = "Click the area you want to go to.";
        locationImg.src = "locationSelectorImages/locationSelectorNone.png";
    }
    if(coralSelected) {
        confirmReefButton.innerText = `Score at Level ${coralLevel} and Area ${numberToLetter[11-(reefArea+4)%12]}`;
    }
}