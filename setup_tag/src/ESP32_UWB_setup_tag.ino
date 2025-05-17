/*
 * ESP32 UWB Ally Communication Sketch
 * Combines UWB distance sensing with ally communication over UART
 * 
 * This ESP32 acts as a UWB tag that connects to UWB anchors for distance measurements.
 * The distance data is then sent to the main ESP32 through the UART (Serial2) connection.
 * The main ESP32 code remains unchanged, as this code formats data in the expected format.
 */

#include <Arduino.h>
#include <SPI.h>

// Include the DW1000 library
#include "DW1000Ranging.h"
#include "DW1000.h"

// UWB pins configuration
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// UART pins for communication with main ESP32
#define UART_TX_PIN 22  // Connect to RX pin of main ESP32 (IO22)
#define UART_RX_PIN 21  // Connect to TX pin of main ESP32
#define UART_BAUD 115200

// Hardware Serial for main ESP32 communication
HardwareSerial MainESP32(2); // Use Serial2 for main ESP32 communication

// TAG antenna delay defaults to 16384
// leftmost two bytes below will become the "short address"
char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

// Variables for UWB data
float currentDistance = 0.0;
float lastSentDistance = 0.0;
String anchorAddress = "";
int signalStrength = 0;
unsigned long lastDistanceUpdate = 0;

// Variables for multiple anchors
#define MAX_ANCHORS 2
String anchorAddresses[MAX_ANCHORS];
float anchorDistances[MAX_ANCHORS];
int anchorSignalStrengths[MAX_ANCHORS];
bool anchorActive[MAX_ANCHORS] = {false, false};
unsigned long anchorLastSeen[MAX_ANCHORS] = {0, 0};

// Leader anchor ID - focus on this one
#define LEADER_ANCHOR_ID "81"
int leaderAnchorIndex = -1; // Index of leader anchor in our arrays

// Z-score filtering parameters
#define BUFFER_SIZE 10  // Number of samples to keep for Z-score calculation
#define Z_THRESHOLD 2.0  // Z-score threshold for outlier detection (2.0 = 95% confidence)
float distanceBuffer[MAX_ANCHORS][BUFFER_SIZE];  // Buffer for each anchor's recent distances
int bufferIndex[MAX_ANCHORS] = {0, 0};  // Current index in buffer for each anchor
bool bufferFilled[MAX_ANCHORS] = {false, false};  // Indicates if buffer has been filled at least once

// Distance between anchors in meters
#define ANCHOR_DISTANCE 1.0

// Counter for messages
int messageCounter = 0;

// Variables for connection tracking
int connectionLossCounter = 0;
int noAnchorCounter = 0;

// Add a reconnection function
void attemptUwbReconnection() {
  Serial.println("Attempting to reconnect UWB...");
  
  // Reinitialize UWB
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  
  // Reattach callback functions
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  
  // Restart ranging
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
  
  // Send status message
  sendStatusMessage("UWB restarted");
  
  // Reset connection tracking
  connectionLossCounter = 0;
  
  // Reset anchor tracking
  for (int i = 0; i < MAX_ANCHORS; i++) {
    anchorActive[i] = false;
    anchorDistances[i] = 0;
    anchorSignalStrengths[i] = 0;
    
    // Reset Z-score buffers
    bufferIndex[i] = 0;
    bufferFilled[i] = false;
    for (int j = 0; j < BUFFER_SIZE; j++) {
      distanceBuffer[i][j] = 0;
    }
  }
  
  // Reset leader anchor index
  leaderAnchorIndex = -1;
}

// Add these variables to store last known information for each anchor
unsigned long lastLeaderUpdate = 0;
unsigned long lastCoLeaderUpdate = 0;
float lastLeaderDistance = 0;
float lastCoLeaderDistance = 0;
float lastLeaderAngle = 0;
float lastCoLeaderAngle = 0;

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Wait a moment for serial to initialize
  delay(1000);
  
  // Print startup message
  Serial.println("\n\n-----------------");
  Serial.println("ESP32 UWB Ally Communication");
  Serial.println("-----------------");
  
  // Initialize UART for communication with main ESP32
  MainESP32.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  
  Serial.println("UART initialized on pins:");
  Serial.print("- TX: ");
  Serial.println(UART_TX_PIN);
  Serial.print("- RX: ");
  Serial.println(UART_RX_PIN);
  
  // Initialize UWB
  Serial.println("Initializing UWB...");
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); // Reset, CS, IRQ pin
  
  // Attach UWB callbacks
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  
  // Start as tag, do not assign random short address
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
  
  Serial.println("UWB initialized as tag");
  
  // Send initial status message to main ESP32
  delay(2000);
  Serial.println("Sending initial status message...");
  sendStatusMessage("Initialized");
}

void loop() {
  // Handle UWB ranging with timing protection
  static unsigned long lastUwbUpdate = 0;
  unsigned long currentTime = millis();
  
  // Ensure UWB ranging gets enough time to operate
  if (currentTime - lastUwbUpdate >= 50) { // UWB update every 50ms (20Hz)
    lastUwbUpdate = currentTime;
    DW1000Ranging.loop();
  }
  
  // Check for commands from main ESP32 less frequently
  static unsigned long lastCommandCheck = 0;
  if (currentTime - lastCommandCheck >= 100) { // Check commands every 100ms
    lastCommandCheck = currentTime;
    checkForCommands();
  }
  
  // Check for stale anchors every second
  static unsigned long lastAnchorCheck = 0;
  if (currentTime - lastAnchorCheck >= 1000) {
    lastAnchorCheck = currentTime;
    
    // Check each anchor to see if it's gone stale (no update in 3 seconds)
    for (int i = 0; i < MAX_ANCHORS; i++) {
      if (anchorActive[i] && (currentTime - anchorLastSeen[i] > 3000)) {
        Serial.print(i == leaderAnchorIndex ? "LEADER " : "CO-LEADER ");
        Serial.print("Anchor ");
        Serial.print(i);
        Serial.println(" has gone stale, marking as inactive");
        
        // Update leader index if this was the leader
        if (i == leaderAnchorIndex) {
          leaderAnchorIndex = -1;
          
          // Check if the other anchor could be the leader
          int otherIndex = (i == 0) ? 1 : 0;
          if (anchorActive[otherIndex] && anchorAddresses[otherIndex] == LEADER_ANCHOR_ID) {
            leaderAnchorIndex = otherIndex;
          }
        }
        
        anchorActive[i] = false;
        
        // Send status update
        String message = (i == leaderAnchorIndex) ? "Lost leader anchor (timeout)" : "Lost co-leader anchor (timeout)";
        sendStatusMessage(message);
      }
    }
  }
  
  // Send periodic updates every 2 seconds
  static unsigned long lastPeriodicUpdate = 0;
  if (currentTime - lastPeriodicUpdate >= 2000) { // Send update every 2 seconds
    lastPeriodicUpdate = currentTime;
    
    int activeAnchors = 0;
    for (int i = 0; i < MAX_ANCHORS; i++) {
      if (anchorActive[i]) activeAnchors++;
    }
    
    // Only send if we have at least one active anchor
    if (activeAnchors > 0) {
      sendDistanceUpdate();
      Serial.println("Sending periodic update (2s interval)");
    }
  }
  
  // If we have no anchor for an extended period, try to reconnect
  static unsigned long lastReconnectCheck = 0;
  if (currentTime - lastReconnectCheck >= 1000) { // Check every second
    lastReconnectCheck = currentTime;
    int activeAnchors = 0;
    for (int i = 0; i < MAX_ANCHORS; i++) {
      if (anchorActive[i]) activeAnchors++;
    }
    
    if (activeAnchors == 0) {
      noAnchorCounter++;
      
      // After 15 seconds of no anchor, try to reconnect
      if (noAnchorCounter >= 15) {
        noAnchorCounter = 0;
        attemptUwbReconnection();
      }
    } else {
      // Reset counter if we have distance readings
      noAnchorCounter = 0;
    }
  }
  
  // Allow some idle time to process other tasks
  delay(5);
}

// Calculate orientation based on distances to two anchors
// Returns array with [x, y, angle1, angle2]
float* calculatePosition(float a1, float a2, float c) {
  static float result[4];
  
  // Using the provided formula
  float b = a1;  // Distance from tag to anchor 1
  float a = c;   // Distance between anchors
  
  // Calculate angle θ using Law of Cosines
  float cos_theta = (b*b + c*c - a2*a2) / (2 * b * c);
  
  // Clamp to valid range to prevent NaN from floating point errors
  if (cos_theta > 1.0) cos_theta = 1.0;
  if (cos_theta < -1.0) cos_theta = -1.0;
  
  float theta = acos(cos_theta);
  
  // Calculate tag position
  float x = b * cos(theta);
  float y = b * sin(theta);
  
  // Calculate angles from anchors
  float theta1 = atan2(y, x) * 180.0 / PI;
  
  float x2 = x - a;
  float theta2 = atan2(y, x2) * 180.0 / PI;
  
  // Store results
  result[0] = x;
  result[1] = y;
  result[2] = theta1;
  result[3] = theta2;
  
  return result;
}

// Calculate Z-score for a distance reading
bool isOutlier(float newDistance, int anchorIndex) {
  // If we don't have enough data yet, can't determine if it's an outlier
  if (!bufferFilled[anchorIndex] && bufferIndex[anchorIndex] < 3) {
    return false;
  }
  
  // Calculate mean
  float sum = 0;
  int count = bufferFilled[anchorIndex] ? BUFFER_SIZE : bufferIndex[anchorIndex];
  
  for (int i = 0; i < count; i++) {
    sum += distanceBuffer[anchorIndex][i];
  }
  
  float mean = sum / count;
  
  // Calculate standard deviation
  float sumSquaredDiff = 0;
  for (int i = 0; i < count; i++) {
    float diff = distanceBuffer[anchorIndex][i] - mean;
    sumSquaredDiff += diff * diff;
  }
  
  float stdDev = sqrt(sumSquaredDiff / count);
  
  // If standard deviation is very small, use a minimum value to avoid division by zero
  // and to prevent rejecting values when there's very little variation
  if (stdDev < 0.01) {
    stdDev = 0.01;
  }
  
  // Calculate Z-score
  float zScore = abs((newDistance - mean) / stdDev);
  
  // Debug
  Serial.print("Z-score for anchor ");
  Serial.print(anchorIndex);
  Serial.print(": ");
  Serial.print(zScore);
  Serial.print(" (mean: ");
  Serial.print(mean);
  Serial.print(", stdDev: ");
  Serial.print(stdDev);
  Serial.print(", new: ");
  Serial.print(newDistance);
  Serial.println(")");
  
  // Return true if it's an outlier
  return zScore > Z_THRESHOLD;
}

// Add a value to the distance buffer
void addToBuffer(float distance, int anchorIndex) {
  // Add to buffer
  distanceBuffer[anchorIndex][bufferIndex[anchorIndex]] = distance;
  
  // Update buffer index
  bufferIndex[anchorIndex] = (bufferIndex[anchorIndex] + 1) % BUFFER_SIZE;
  
  // Mark as filled if we've gone through the buffer once
  if (bufferIndex[anchorIndex] == 0) {
    bufferFilled[anchorIndex] = true;
  }
}

// UWB new range callback
void newRange() {
  // Get a handle to the device
  DW1000Device *device = DW1000Ranging.getDistantDevice();
  
  // Only update if we have a valid device
  if (device) {
    String address = String(device->getShortAddress(), HEX);
    float distance = device->getRange();
    int rxPower = device->getRXPower();
    
    // Check if this is our leader anchor
    bool isLeader = (address == LEADER_ANCHOR_ID);
    
    // Find the anchor in our array or add it if new
    int anchorIndex = -1;
    
    // First, check if it's already in our array
    for (int i = 0; i < MAX_ANCHORS; i++) {
      if (anchorActive[i] && anchorAddresses[i] == address) {
        anchorIndex = i;
        break;
      }
    }
    
    // If not found but it's the leader, give it priority as index 0
    if (anchorIndex == -1 && isLeader) {
      // If slot 0 is taken, move its data to slot 1
      if (anchorActive[0]) {
        // Only move if slot 1 is empty or not the leader
        if (!anchorActive[1] || anchorAddresses[1] != LEADER_ANCHOR_ID) {
          anchorAddresses[1] = anchorAddresses[0];
          anchorDistances[1] = anchorDistances[0];
          anchorSignalStrengths[1] = anchorSignalStrengths[0];
          anchorActive[1] = true;
          anchorLastSeen[1] = anchorLastSeen[0];
          
          // Also move the buffer data
          for (int i = 0; i < BUFFER_SIZE; i++) {
            distanceBuffer[1][i] = distanceBuffer[0][i];
          }
          bufferIndex[1] = bufferIndex[0];
          bufferFilled[1] = bufferFilled[0];
        }
      }
      anchorIndex = 0;
      leaderAnchorIndex = 0;
    }
    // If not found and not leader, use the first empty slot or slot 1
    else if (anchorIndex == -1) {
      // If leader is in slot 0, use slot 1
      if (leaderAnchorIndex == 0) {
        anchorIndex = 1;
      }
      // If no leader yet, or leader is in slot 1, use first empty slot
      else {
        for (int i = 0; i < MAX_ANCHORS; i++) {
          if (!anchorActive[i]) {
            anchorIndex = i;
            break;
          }
        }
        // If no empty slots, use slot 1 (non-leader slot)
        if (anchorIndex == -1) {
          anchorIndex = 1;
        }
      }
    }
    
    // If this is the leader, update leader index
    if (isLeader) {
      leaderAnchorIndex = anchorIndex;
    }
    
    // If we found a place for this anchor
    if (anchorIndex >= 0) {
      // Check for outliers if the anchor is already active
      bool outlier = false;
      if (anchorActive[anchorIndex]) {
        outlier = isOutlier(distance, anchorIndex);
      }
      
      // Don't update if it's an outlier
      if (!outlier) {
        // Add to buffer
        addToBuffer(distance, anchorIndex);
        
        // Simple low-pass filter to reduce jitter
        const float FILTER_FACTOR = 0.3; // Adjust between 0-1 (higher = less filtering)
        
        if (!anchorActive[anchorIndex]) {
          // First reading for this anchor
          anchorDistances[anchorIndex] = distance;
          anchorAddresses[anchorIndex] = address;
          anchorActive[anchorIndex] = true;
        } else {
          // Apply filter
          anchorDistances[anchorIndex] = (FILTER_FACTOR * distance) + ((1-FILTER_FACTOR) * anchorDistances[anchorIndex]);
        }
        
        // Update signal strength and last seen time
        anchorSignalStrengths[anchorIndex] = rxPower;
        anchorLastSeen[anchorIndex] = millis();
        
        // Debug print with leader designation
        Serial.print(isLeader ? "LEADER " : "CO-LEADER ");
        Serial.print("Anchor ");
        Serial.print(anchorIndex);
        Serial.print(": ");
        Serial.print(address);
        Serial.print(", Distance: ");
        Serial.print(anchorDistances[anchorIndex]);
        Serial.print(" m, Signal: ");
        Serial.print(anchorSignalStrengths[anchorIndex]);
        Serial.println(" dBm");
        
        // Also update the legacy variables for backward compatibility
        // Always prioritize the leader anchor if available
        if (leaderAnchorIndex >= 0 && anchorActive[leaderAnchorIndex]) {
          currentDistance = anchorDistances[leaderAnchorIndex];
          anchorAddress = anchorAddresses[leaderAnchorIndex];
          signalStrength = anchorSignalStrengths[leaderAnchorIndex];
        } else {
          // Fall back to the first active anchor if leader not available
          for (int i = 0; i < MAX_ANCHORS; i++) {
            if (anchorActive[i]) {
              currentDistance = anchorDistances[i];
              anchorAddress = anchorAddresses[i];
              signalStrength = anchorSignalStrengths[i];
              break;
            }
          }
        }
      } else {
        Serial.print("Outlier detected for anchor ");
        Serial.print(anchorIndex);
        Serial.print(": ");
        Serial.print(distance);
        Serial.println(" (ignored)");
      }
    }
  }
}

// UWB new device callback
void newDevice(DW1000Device *device) {
  Serial.print("New device detected: ");
  Serial.println(device->getShortAddress(), HEX);
  
  // Check if it's the leader
  String address = String(device->getShortAddress(), HEX);
  bool isLeader = (address == LEADER_ANCHOR_ID);
  
  // Send notification to main ESP32
  String message = isLeader ? "New leader anchor connected" : "New co-leader anchor connected";
  sendStatusMessage(message);
}

// UWB inactive device callback
void inactiveDevice(DW1000Device *device) {
  // Get the address of the inactive device
  String address = String(device->getShortAddress(), HEX);
  
  // Find which anchor this is
  for (int i = 0; i < MAX_ANCHORS; i++) {
    if (anchorActive[i] && anchorAddresses[i] == address) {
      // Log the inactive device
      Serial.print(i == leaderAnchorIndex ? "LEADER " : "CO-LEADER ");
      Serial.print("Anchor ");
      Serial.print(i);
      Serial.print(" inactive: ");
      Serial.println(address);
      
      // Mark anchor as inactive
      anchorActive[i] = false;
      
      // Send notification to main ESP32
      String message = (i == leaderAnchorIndex) ? "Lost leader anchor" : "Lost co-leader anchor";
      sendStatusMessage(message);
      
      break;
    }
  }
  
  // Check if we have any active anchors left
  bool anyActive = false;
  for (int i = 0; i < MAX_ANCHORS; i++) {
    if (anchorActive[i]) {
      anyActive = true;
      break;
    }
  }
  
  // If all anchors are lost, reset legacy variables
  if (!anyActive) {
    currentDistance = 0;
    anchorAddress = "";
    signalStrength = 0;
  }
}

void checkForCommands() {
  // Check if data is available from main ESP32
  while (MainESP32.available()) {
    String receivedData = MainESP32.readStringUntil('\n');
    
    // Trim any whitespace or newlines
    receivedData.trim();
    
    Serial.print("Received command: '");
    Serial.print(receivedData);
    Serial.println("'");
    
    // Process commands
    if (receivedData == "REQUEST_DATA") {
      Serial.println("Data request received, sending latest distance update...");
      sendDistanceUpdate(); // Send the latest data immediately
    }
    
    // Add more command processing here if needed
  }
}

void sendDistanceUpdate() {
  messageCounter++;
  
  int activeAnchors = 0;
  for (int i = 0; i < MAX_ANCHORS; i++) {
    if (anchorActive[i]) activeAnchors++;
  }
  
  // Format: "ALLY|Sender|Message|Timestamp|SignalStrength"
  String sender = "UWB-Tag";
  String message;
  
  // Find leader and co-leader indexes
  int leaderIdx = -1;
  int coLeaderIdx = -1;
  
  if (leaderAnchorIndex >= 0 && anchorActive[leaderAnchorIndex]) {
    leaderIdx = leaderAnchorIndex;
    // Update last known leader information
    lastLeaderDistance = anchorDistances[leaderIdx];
    lastLeaderUpdate = millis();
  }
  
  // Find co-leader (first non-leader active anchor)
  for (int i = 0; i < MAX_ANCHORS; i++) {
    if (anchorActive[i] && i != leaderIdx) {
      coLeaderIdx = i;
      // Update last known co-leader information
      lastCoLeaderDistance = anchorDistances[coLeaderIdx];
      lastCoLeaderUpdate = millis();
      break;
    }
  }
  
  // Calculate position and angles if both anchors are active
  float leader_angle = 0;
  float coleader_angle = 0;
  
  if (leaderIdx >= 0 && coLeaderIdx >= 0) {
    float* result = calculatePosition(
      anchorDistances[leaderIdx],
      anchorDistances[coLeaderIdx],
      ANCHOR_DISTANCE
    );
    
    leader_angle = result[2];
    coleader_angle = result[3];
    
    // Update last known angles
    lastLeaderAngle = leader_angle;
    lastCoLeaderAngle = coleader_angle;
  }
  
  // Build proper JSON message
  message = "{";
  
  // Add leader information (use last known data if not active)
  message += "\"Leader\":{";
  if (leaderIdx >= 0) {
    message += "\"distance\":" + String(anchorDistances[leaderIdx], 2);
    if (coLeaderIdx >= 0) {
      message += ",\"angle\":" + String(leader_angle, 1);
    }
  } else if (lastLeaderUpdate > 0) {
    // Use last known values but mark as stale
    message += "\"distance\":" + String(lastLeaderDistance, 2) + ",\"stale\":true";
    if (lastCoLeaderUpdate > 0) {
      message += ",\"angle\":" + String(lastLeaderAngle, 1);
    }
  } else {
    message += "\"detected\":false";
  }
  message += "},";
  
  // Add co-leader information with proper JSON syntax and hyphenated name
  message += "\"Co-leader\":{";
  
  if (coLeaderIdx >= 0) {
    message += "\"distance\":" + String(anchorDistances[coLeaderIdx], 2);
    if (leaderIdx >= 0) {
      message += ",\"angle\":" + String(coleader_angle, 1);
    }
  } else if (lastCoLeaderUpdate > 0) {
    // Use last known values but mark as stale
    message += "\"distance\":" + String(lastCoLeaderDistance, 2) + ",\"stale\":true";
    if (lastLeaderUpdate > 0) {
      message += ",\"angle\":" + String(lastCoLeaderAngle, 1);
    }
  } else {
    message += "\"detected\":false";
  }
  message += "},";
  
  // Add status information
  message += "\"Status\":\"";
  
  if (activeAnchors == 0) {
    message += "No anchors";
  } else if (leaderIdx < 0 && lastLeaderUpdate > 0) {
    message += "Lost leader";
  } else if (coLeaderIdx < 0 && lastCoLeaderUpdate > 0) {
    message += "Lost co-leader";
  } else if (activeAnchors == 2) {
    message += "All active";
  } else if (leaderIdx >= 0) {
    message += "Leader only";
  } else if (coLeaderIdx >= 0) {
    message += "Co-leader only";
  } else {
    message += "OK";
  }
  
  message += "\"";
  message += "}";
  
  // Format complete message
  String completeMessage = "ALLY|" + sender + "|" + message + 
                          "|" + String(millis()/1000) + "s|";
                           
  // Use signal strength from leader if available, otherwise highest signal
  if (leaderIdx >= 0) {
    completeMessage += String(anchorSignalStrengths[leaderIdx]);
  } else if (coLeaderIdx >= 0) {
    completeMessage += String(anchorSignalStrengths[coLeaderIdx]);
  } else if (activeAnchors > 0) {
    int maxSignal = -200;  // Start with very low value
    for (int i = 0; i < MAX_ANCHORS; i++) {
      if (anchorActive[i] && anchorSignalStrengths[i] > maxSignal) {
        maxSignal = anchorSignalStrengths[i];
      }
    }
    completeMessage += String(maxSignal);
  } else {
    completeMessage += "0";
  }
  
  // Send to main ESP32
  MainESP32.println(completeMessage);
  MainESP32.flush(); // Ensure the message is sent completely
  
  // Log to debug serial
  Serial.print("Sent: ");
  Serial.println(completeMessage);
}

void sendStatusMessage(String status) {
  messageCounter++;
  
  // Format: "ALLY|Sender|Message|Timestamp|SignalStrength"
  String sender = "UWB-Tag";
  
  // Build proper JSON message with all fields
  String message = "{";
  
  // Add leader information
  message += "\"Leader\":{";
  if (leaderAnchorIndex >= 0 && anchorActive[leaderAnchorIndex]) {
    message += "\"distance\":" + String(anchorDistances[leaderAnchorIndex], 2);
    // Include angle if we have co-leader data
    bool hasCoLeader = false;
    for (int i = 0; i < MAX_ANCHORS; i++) {
      if (anchorActive[i] && i != leaderAnchorIndex) {
        hasCoLeader = true;
        break;
      }
    }
    if (hasCoLeader && lastLeaderAngle != 0) {
      message += ",\"angle\":" + String(lastLeaderAngle, 1);
    }
  } else if (lastLeaderUpdate > 0) {
    message += "\"distance\":" + String(lastLeaderDistance, 2) + ",\"stale\":true";
    if (lastLeaderAngle != 0) {
      message += ",\"angle\":" + String(lastLeaderAngle, 1);
    }
  } else {
    message += "\"detected\":false";
  }
  message += "},";
  
  // Add co-leader information with hyphenated name
  message += "\"Co-leader\":{";
  
  // Find co-leader if any
  int coLeaderIdx = -1;
  for (int i = 0; i < MAX_ANCHORS; i++) {
    if (anchorActive[i] && i != leaderAnchorIndex) {
      coLeaderIdx = i;
      break;
    }
  }
  
  if (coLeaderIdx >= 0) {
    message += "\"distance\":" + String(anchorDistances[coLeaderIdx], 2);
    if (lastCoLeaderAngle != 0) {
      message += ",\"angle\":" + String(lastCoLeaderAngle, 1);
    }
  } else if (lastCoLeaderUpdate > 0) {
    message += "\"distance\":" + String(lastCoLeaderDistance, 2) + ",\"stale\":true";
    if (lastCoLeaderAngle != 0) {
      message += ",\"angle\":" + String(lastCoLeaderAngle, 1);
    }
  } else {
    message += "\"detected\":false";
  }
  message += "},";
  
  // Add the status message
  message += "\"Status\":\"" + status + "\"";
  message += "}";
  
  // Format complete message
  String completeMessage = "ALLY|" + sender + "|" + message + 
                          "|" + String(millis()/1000) + "s|100";
  
  // Send to main ESP32
  MainESP32.println(completeMessage);
  MainESP32.flush(); // Ensure the message is sent completely
  
  // Log to debug serial
  Serial.print("Sent status: ");
  Serial.println(completeMessage);
}
