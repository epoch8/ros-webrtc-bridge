var ros = new ROSLIB.Ros();

ros.connect("ws://192.168.0.27:9090");

var offer_svc = new ROSLIB.Service({
    ros: ros, 
    name: "/webrtc_bridge_local_node/offer", 
    serviceType: "webrtc_bridge_srv/srv/WebRTCOffer",
});

var pc = null;

function negotiate() {
    pc.addTransceiver('video', { direction: 'recvonly' });
    pc.addTransceiver('audio', { direction: 'recvonly' });
    return pc.createOffer().then((offer) => {
        return pc.setLocalDescription(offer);
    }).then(() => {
        // wait for ICE gathering to complete
        return new Promise((resolve) => {
            if (pc.iceGatheringState === 'complete') {
                resolve();
            } else {
                const checkState = () => {
                    if (pc.iceGatheringState === 'complete') {
                        pc.removeEventListener('icegatheringstatechange', checkState);
                        resolve();
                    }
                };
                pc.addEventListener('icegatheringstatechange', checkState);
            }
        });
    }).then(() => {
        var offer = pc.localDescription;
        return new Promise((resolve, reject) => {
            offer_svc.callService(new ROSLIB.ServiceRequest({
                sdp: offer.sdp,
                type: offer.type,
            }), (response) => {
                if (response) {
                    resolve(response);
                } else {
                    reject(new Error("Failed to get response"));
                }
            });
        });
    }).then((response) => {
        // response is a ROSLIB.ServiceResponse object
        return pc.setRemoteDescription(response);
    }).catch((e) => {
        alert(e);
    });
}

function start() {
    var config = {
        sdpSemantics: 'unified-plan'
    };

    if (document.getElementById('use-stun').checked) {
        config.iceServers = [{ urls: ['stun:stun.l.google.com:19302'] }];
    }

    pc = new RTCPeerConnection(config);

    // connect audio / video
    pc.addEventListener('track', (evt) => {
        if (evt.track.kind == 'video') {
            document.getElementById('video').srcObject = evt.streams[0];
        } else {
            document.getElementById('audio').srcObject = evt.streams[0];
        }
    });

    document.getElementById('start').style.display = 'none';
    negotiate();
    document.getElementById('stop').style.display = 'inline-block';
}

function stop() {
    document.getElementById('stop').style.display = 'none';

    // close peer connection
    setTimeout(() => {
        pc.close();
    }, 500);
}
