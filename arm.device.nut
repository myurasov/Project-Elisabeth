agent.on("command", function(command) {
  server.log("Received " + command);
})

function init() {
    hardware.uart12.configure(115200, 8, PARITY_NONE, 1, NO_CTSRTS);
    command("#5p1500 #6p2430 #4p2350 #3p500 #2p1950 #1p1350 t2000");
    
    agent.on("command", function(data) {
      command(data);
    })
}

function command(cmd) {
  server.log("command: " + cmd);
  hardware.uart12.write(cmd + "\r");
}
 
init();