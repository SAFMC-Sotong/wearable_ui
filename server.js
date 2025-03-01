// UDP to WebSocket bridge server
// Save this as server.js and run with: node server.js
const dgram = require('dgram');
const WebSocket = require('ws');
const http = require('http');
const fs = require('fs');
const path = require('path');

// Create UDP server to listen for drone commands
const UDP_PORT = 7880;  // The port that your Python code is sending to
const udpServer = dgram.createSocket('udp4');

// Create HTTP server to serve the HTML page
const server = http.createServer((req, res) => {
  // Serve the HTML file
  if (req.url === '/' || req.url === '/index.html') {
    fs.readFile(path.join(__dirname, 'index.html'), (err, data) => {
      if (err) {
        res.writeHead(500);
        res.end('Error loading index.html');
        console.error('Error loading index.html:', err);
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
const clients = new Set();

// Handle WebSocket connections
wss.on('connection', (ws) => {
  // Add new client to the set
  clients.add(ws);
  console.log('WebSocket client connected');
  
  // Send initial message to confirm connection
  ws.send('SYSTEM CONNECTED');
  
  // Handle client messages (if needed)
  ws.on('message', (message) => {
    console.log('Received from client:', message.toString());
  });
  
  // Handle disconnection
  ws.on('close', () => {
    clients.delete(ws);
    console.log('WebSocket client disconnected');
  });
  
  // Handle errors
  ws.on('error', (error) => {
    console.error('WebSocket error:', error);
    clients.delete(ws);
  });
});

// Handle UDP messages
udpServer.on('message', (msg, rinfo) => {
  const message = msg.toString('utf8').trim();
  console.log(`UDP message from ${rinfo.address}:${rinfo.port}: ${message}`);
  
  // Broadcast the message to all connected WebSocket clients
  for (const client of clients) {
    if (client.readyState === WebSocket.OPEN) {
      try {
        client.send(message);
      } catch (error) {
        console.error('Error sending to WebSocket client:', error);
        clients.delete(client);
      }
    }
  }
});

// Start UDP server
udpServer.on('listening', () => {
  const address = udpServer.address();
  console.log(`UDP server listening on ${address.address}:${address.port}`);
});

udpServer.on('error', (error) => {
  console.error('UDP server error:', error);
});

// Bind UDP server to port
try {
  udpServer.bind(UDP_PORT);
} catch (error) {
  console.error('Failed to bind UDP server:', error);
  process.exit(1);
}

// Start HTTP server
const PORT = 8080;
server.listen(PORT, () => {
  console.log(`HTTP/WebSocket server running on http://localhost:${PORT}`);
});

// Send a heartbeat to all clients every 30 seconds to keep connections alive
setInterval(() => {
  const timestamp = new Date().toISOString();
  const heartbeat = `HEARTBEAT ${timestamp}`;
  
  let activeClients = 0;
  for (const client of clients) {
    if (client.readyState === WebSocket.OPEN) {
      try {
        client.send(heartbeat);
        activeClients++;
      } catch (error) {
        console.error('Error sending heartbeat:', error);
        clients.delete(client);
      }
    }
  }
  
  console.log(`Sent heartbeat to ${activeClients} clients`);
}, 30000);

// Handle shutdown gracefully
process.on('SIGINT', () => {
  console.log('Shutting down servers...');
  
  // Close all WebSocket connections
  for (const client of clients) {
    try {
      client.close();
    } catch (e) {
      // Ignore errors on close
    }
  }
  
  // Close servers
  udpServer.close(() => {
    console.log('UDP server closed');
  });
  
  server.close(() => {
    console.log('HTTP/WebSocket server closed');
    process.exit(0);
  });
  
  // Force exit after timeout if servers don't close cleanly
  setTimeout(() => {
    console.log('Forcing exit after timeout');
    process.exit(1);
  }, 3000);
});
