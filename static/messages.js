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
//   mt_<measurement name>  --> Looks up a string from value depending on the measurement name

let socket = null;
let disconnected = true;
let reconnectTimer = null;

// Global AmIdFilter is used to filter aruco marker ids
let AmIdFilter = -1; // Negative for no filter

function messages_startSocket(eventName, onMessage, onConnect) {
    if (!socket) {
        socket = io();

        socket.on('connect', () => {
            disconnected = false;
            if (typeof onConnect === 'function') {
                onConnect();
            }
        });

        socket.on('disconnect', () => disconnected = true);
        socket.on('connect_error', () => disconnected = true);

        if (reconnectTimer === null) {
            reconnectTimer = setInterval(() => {
                if (disconnected) {
                    socket.connect();
                }
            }, 5000);
        }
    }

    socket.on(eventName, data => {
        onMessage(JSON.parse(data));
    });
}

// updateId function (call from your onMessage callback)
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

        el = document.getElementById("mt_" + key);
        if (el) el.innerHTML = to_string(key,parseInt(value));
    }
}

// Export AmIdFilter if needed elsewhere
window.AmIdFilter = AmIdFilter;
window.startSocket = messages_startSocket;
window.updateId = messages_updateId;

// ------------------------------------------------------------------------------
//
// Translation of integer values to strings for mt_
//
// ------------------------------------------------------------------------------
const GPS_MODE_STRINGS = ["None", "Search", "Doppler", "SPS", "Differential", "RTK float", "RTK integer",  // 0..6
    "WAAS", "OmniSTAR", "OmniSTAR HP", "No data", "Blanked", "Doppler(PP)", "SPS(PP)", "Differential(PP)", // 7..14
    "RTK float(PP)", "RTK integer(PP)", "OmniStar XP", "CDGPS", "Not recognised", "gxDoppler", "gxSPS",    // 15..21
    "gxDifferential", "gxFloat", "gxInteger", "ixDoppler", "ixSPS", "ixDifferential", "ixFloat",           // 22..28
    "ixInteger", "PPP converging", "PPP", "Unknown", "Unknown", "GAD" // 29..34
];
const HEADING_QUALITY_STRINGS = ["None", "Poor", "OK", "Good"];

const STRING_MAP = {
  GpsPosMode: GPS_MODE_STRINGS,
  GpsVelMode: GPS_MODE_STRINGS,
  GpsAttMode: GPS_MODE_STRINGS,
  HeadingQuality: HEADING_QUALITY_STRINGS,
  // Add more keys here as needed
};

function to_string(key, index) {
  const list = STRING_MAP[key];
  return (list && index >= 0 && index < list.length) ? `${list[index]} (${index})` : index;
}
