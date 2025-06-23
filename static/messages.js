// messages.js - Socket.IO version
// Processes messages from the host and updates elements with matching id names.
//
// Usage: 
//   startSocket('event_name', onMessageCallback);
//   function onMessageCallback(data) { ... }
//
// Matching id names:
//   mi_<measurement name>  --> update innerHTML using integer value
//   ms_<measurement name>  --> update innerHTML using string
//   mf#_<measurement name> --> update innerHTML with # decimal places

let socket = null;
let disconnected = true;
let reconnectTimer = null;

// Global AmIdFilter is used to filter aruco marker ids
let AmIdFilter = -1; // Negative for no filter

function messages_startSocket(eventName, onMessage) {
    if (!socket) {
        socket = io();
        socket.on('connect', () => disconnected = false);
        socket.on('disconnect', () => disconnected = true);
        socket.on('connect_error', () => disconnected = true);

        if (reconnectTimer === null) {
            reconnectTimer = setInterval(() => {
                if (disconnected) {
                    socket.connect(); // Attempt to reconnect existing socket
                }
            }, 5000);
        }
    }

    socket.on(eventName, data => {
        onMessage(JSON.parse(data));
    });
}

// Example updateId function (call from your onMessage callback)
function messages_updateId(data) {
    // Example: update elements with ids like mi_GpsDiffAge, ms_BaseStationId, mf3_Roll
    for (const key in data) {
        if (!data.hasOwnProperty(key)) continue;
        let value = data[key];

        // Integer value
        let el = document.getElementById("mi_" + key);
        if (el) el.innerHTML = parseInt(value);

        // String value
        el = document.getElementById("ms_" + key);
        if (el) el.innerHTML = value;

        // Float with N decimals: look for ids like mf2_<key>, mf3_<key>, etc.
        for (let d = 1; d <= 9; d++) {
            el = document.getElementById("mf" + d + "_" + key);
            if (el) el.innerHTML = parseFloat(value).toFixed(d);
        }
    }
}

// Export AmIdFilter if needed elsewhere
window.AmIdFilter = AmIdFilter;
window.startSocket = messages_startSocket;
window.updateId = messages_updateId;


