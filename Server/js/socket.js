var connection = new WebSocket('wss://protopaja2.research.comnet.aalto.fi/ws', ['soap', 'xmpp']);

// Identify as browser client to the websocket server
connection.onopen = function() {
  connection.send('browser');
};

// Log errors
connection.onerror = function(error) {
  console.log('WebSocket Error ' + error);
};

// Rotate bones based on received websocket messages
connection.onmessage = function(e) {
  data = JSON.parse(e.data);
  console.log(data);
  Object.keys(data).forEach((axis, index) => {
    if (axis != "bone_name")
      animator.sensor_update_degree(data.bone_name, axis, data[axis]);
  });
  animator.rotate_all();
};