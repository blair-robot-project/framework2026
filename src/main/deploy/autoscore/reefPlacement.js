const locationImg = document.getElementById("locationSelect");
const confirmReefButton = document.getElementById("confirmReefButton");

["touchmove", "mousemove"].forEach(_ => {
    locationImg.addEventListener(_, (event) => {
        if(!pairSelected) {
            let imgPosition = locationImg.getBoundingClientRect();
            let imageCenterX = imgPosition.left + (imgPosition.right-imgPosition.left) / 2;
            let imageCenterY = imgPosition.top + (imgPosition.bottom-imgPosition.top) / 2;
            
            let x = event.clientX-imageCenterX;
            let y = imageCenterY-event.clientY;
            if(_ == "touchmove") {
                event.preventDefault();
                x = event.changedTouches[0].clientX-imageCenterX;
                y = imageCenterY-event.changedTouches[0].clientY;
            }
    
            let angleRadians = Math.atan2(x, y);
            let angleDegrees = angleRadians * (180 / Math.PI);
            if(angleRadians < 0) {
                angleDegrees = 360 + angleRadians * (180 / Math.PI);
            }
            reefPair = (Math.floor(((-angleDegrees+210+360) % 360)/60) % 6) + 1;
            if(reefPair == -1) {
                locationImg.src = `locationSelectorImages/locationSelectorNone.png`;
            } else {
                locationImg.src = `locationSelectorImages/locationSelectorPair${reefPair}.png`;
            }
        }
    });
});

["touchend", "mouseleave"].forEach(_ => {
    locationImg.addEventListener(_, () => {
        if(!pairSelected) {
            locationImg.src = "locationSelectorImages/locationSelectorNone.png";
        }
    });
});


locationImg.onclick = () => {
    pairSelected = !pairSelected;
    if(pairSelected) {
        if(coralSelected) {
            confirmReefButton.style.backgroundColor = "rgb(4, 189, 36)";
            confirmReefButton.innerText = `Score at Level ${coralLevel} and Pair ${reefPair}`;
        } else {
            confirmReefButton.innerText = `Reef Pair: ${reefPair}`;
        }
    } else {
        reefPair = -1;
        if(coralSelected) {
            confirmReefButton.innerText = `Coral Level: L${coralLevel}`;
        } else {
            confirmReefButton.innerText = `Choose Robot Alignment`;
        }
        locationImg.src = "locationSelectorImages/locationSelectorNone.png";
        confirmReefButton.style.backgroundColor = "#DD0000";
    }
    
}