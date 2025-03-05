// server.js - Backend server for TEAM SOTONG DRONE ALPHA wearable UI interface
// listens for UDP data and forwards it to connected web clients via WebSockets

const dgram = require('dgram');
const WebSocket = require('ws');
const http = require('http');
const fs = require('fs');
const path = require('path');

// Configuration
const UDP_PORT = 7880;           // Port to listen for UDP data
const WEB_PORT = 8080;           // Port for WebSocket and HTTP server
const UDP_LISTEN_ADDR = '0.0.0.0'; // Listen on all interfaces

// Create UDP server
const udpServer = dgram.createSocket('udp4');

// Create HTTP server to serve the HTML page
const server = http.createServer((req, res) => {
  if (req.url === '/') {
    // Serve the HTML file
    fs.readFile(path.join(__dirname, 'index.html'), (err, data) => {
      if (err) {
        res.writeHead(500);
        res.end('Error loading index.html');
        return;
      }
      res.writeHead(200, { 'Content-Type': 'text/html' });
      res.end(data);
    });
  } else {
    res.writeHead(404);
    res.end('Not found');
  }
});

// Create WebSocket server
const wss = new WebSocket.Server({ server });

// Store connected WebSocket clients
const clients = new Set();

// Handle new WebSocket connections
wss.on('connection', (ws) => {
  console.log('Client connected');
  clients.add(ws);
  
  // Handle client disconnection
  ws.on('close', () => {
    console.log('Client disconnected');
    clients.delete(ws);
  });
  
  // Handle errors
  ws.on('error', (error) => {
    console.error('WebSocket error:', error);
    clients.delete(ws);
  });
  
  // Send initial message
  ws.send(JSON.stringify({
    type: 'udpData',
    message: 'Waiting for drone data...'
  }));
});

// Set up UDP server
udpServer.on('error', (err) => {
  console.error(`UDP server error:\n${err.stack}`);
  udpServer.close();
});

// Parse UDP packets from the drone
udpServer.on('message', (msg, rinfo) => {
  console.log(`Received ${msg.length} bytes from ${rinfo.address}:${rinfo.port}`);
  
  try {
    // Convert Buffer to string
    const dataStr = msg.toString('utf8');
    console.log(`Data: ${dataStr}`);
    
    // Parse the message - format may vary based on your drone's protocol
    // This is a simple example assuming the drone sends plain text data
    
    // Check for battery information
    const batteryMatch = dataStr.match(/BAT[:]?\s*(\d+)/i);
    if (batteryMatch && batteryMatch[1]) {
      const batteryLevel = parseInt(batteryMatch[1], 10);
      
      // Send battery update to all clients
      broadcastToAll({
        type: 'batteryLevel',
        level: batteryLevel
      });
    }
    
    // Send the data to all connected WebSocket clients
    broadcastToAll({
      type: 'udpData',
      message: dataStr.trim()
    });
  } catch (error) {
    console.error('Error processing UDP message:', error);
  }
});

// Function to broadcast message to all connected WebSocket clients
function broadcastToAll(data) {
  const message = JSON.stringify(data);
  
  clients.forEach((client) => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(message);
    }
  });
}

// Start the UDP server
udpServer.bind(UDP_PORT, UDP_LISTEN_ADDR, () => {
  const address = udpServer.address();
  console.log(`UDP server listening on ${address.address}:${address.port}`);
});

// Start the WebSocket/HTTP server
server.listen(WEB_PORT, () => {
  console.log(`WebSocket/HTTP server started on port ${WEB_PORT}`);
});

// Handle graceful shutdown
process.on('SIGINT', () => {
  console.log('Shutting down servers...');
  
  // Close WebSocket server
  wss.close(() => {
    console.log('WebSocket server closed');
    
    // Close HTTP server
    server.close(() => {
      console.log('HTTP server closed');
      
      // Close UDP server
      udpServer.close(() => {
        console.log('UDP server closed');
        process.exit(0);
      });
    });
  });
});