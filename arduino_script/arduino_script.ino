#include <Dynamixel2Arduino.h>
#include <tuple>
using std::tuple;
using std::make_tuple;
using std::get;

#if defined(ARDUINO_OpenRB)  // When using OpenRB-150
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#endif

tuple<int, int> homeMotor(int DXL_ID);

const uint8_t DXL_IDs[] = {1, 2, 3, 4};
const int NUM_MOTORS = sizeof(DXL_IDs) / sizeof(DXL_IDs[0]);
const float DXL_PROTOCOL_VERSION = 2.0;
const int DXL_BAUD_RATE = 57600;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

// Max current
const float MAX_CURRENT = 300; // DO NOT Exceed 300 for precise bipolar forceps, NEVER over 500 for any instrument

// Max range for control wheels (in degrees). (also max turn for finding the second motor limits)
const float MAX_POSITION_RANGE = 600; 

// Convention for direction of rotation
const bool POSITIVE_IS_CCW = true;

// Motor range limits (will be set after homing)
int cw_limits[NUM_MOTORS];
int ccw_limits[NUM_MOTORS];

// Transmission ratio matrix and inverse
double transmissionMatrix[4][4] = {
  {0, 0, 0, 1.54},
  {0, 0, 1.03, 0},
  {0, 1.15, -0.743, 0},
  {1.15, 0, -0.743, 0}
};
double inverseMatrix[4][4] = {
  {0, 0.62726888982693119459, 0, 0.86956521739130434782},
  {0, 0.62726888982693119459, 0.86956521739130434782, 0},
  {0, 0.97087378640776699029, 0, 0},
  {0.64935064935064935064, 0, 0, 0}
};

int iters = 0;

// Define the desired end-effector angles in degrees
double theta_R = 0; 
double theta_W = 0;
double theta_G1 = -1;
double theta_G2 = 3; //can reverse direction of gripper 2 angle for usablity (need to change intersection check)
// theta_G1 and theta_G2 != 0 to make sure the gripper is closed at the beginning

void setup() {
  delay(1000);
  Serial.begin(57600);
  
  dxl.begin(DXL_BAUD_RATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Initialize all motors
  for (int i = 0; i < NUM_MOTORS; i++) {
    int DXL_ID = DXL_IDs[i];
    if (dxl.ping(DXL_ID)) {
      Serial.print("Dynamixel with ID: ");
      Serial.print(DXL_ID);
      Serial.println(" is connected!");

      // Set to Current Control Mode for homing and limit finding
      dxl.torqueOff(DXL_ID);
      dxl.setOperatingMode(DXL_ID, OP_CURRENT);
      dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID, MAX_CURRENT);
      dxl.writeControlTableItem(HOMING_OFFSET, DXL_ID, 0);

      // Perform homing for this motor
      tuple<int,int> limits = homeMotor(DXL_ID);
      
      // Store the motor's CW and CCW position limits after homing
      cw_limits[i] =  get<0>(limits);
      ccw_limits[i] = get<1>(limits);
    } 
    else {
      Serial.print("Failed to connect to Dynamixel with ID: ");
      Serial.println(DXL_ID);
    }
  }
}


void loop() {
//  V1 (Aban)
//  if (Serial.available() > 0) {
//    String receivedData = Serial.readStringUntil('\n');  // Read until newline
//    processAngles(receivedData);
//  }
  

  if (Serial.available() >= 16) 
  {  // Adjust depending on your protocol
    float received_angles[4];
    uint8_t raw_bytes[16];
  
    for (int i = 0; i < 16; i++) {
      raw_bytes[i] = Serial.read();
    }
  
    // Convert 4 x 4-byte segments to floats
    for (int i = 0; i < 4; i++) {
      memcpy(&received_angles[i], &raw_bytes[i * 4], sizeof(float));
    }
    
    theta_R  = received_angles[0];
    theta_W  = received_angles[1];
    theta_G1 = received_angles[2];
    theta_G2 = received_angles[3];
  }
  
  // Gripper intersection check
  if (theta_G2 - 12 > theta_G1){
    Serial.println("Error: Gripper angles cannot intersect, midpoint chosen instead");
    theta_G1 = (theta_G1+theta_G2)/2 - 5;
    theta_G2 = theta_G1;
  }

  // Convert degrees to radians
  double theta_R_rad = theta_R * PI / 180.0;
  double theta_W_rad = theta_W * PI / 180.0;
  double theta_G1_rad = theta_G1 * PI / 180.0;
  double theta_G2_rad = theta_G2 * PI / 180.0; 
  // Desired end-effector angles vector
  double desiredAngles[4] = {theta_R_rad, theta_W_rad, theta_G1_rad, theta_G2_rad};

  // Calculate necessary motor angles using transmission matrix
  double motorAngles[4] = {0};

  for (int i = 0; i < 4; i++) {
    motorAngles[i] = 0; // Initialize to zero before summing
    for (int j = 0; j < 4; j++) {
      motorAngles[i] += inverseMatrix[i][j] * desiredAngles[j];
    }
  }

  // Convert radians to degrees
  for (int i = 0; i < 4; i++) {
    motorAngles[i] *= 180.0 / PI;
  }

  // Send the calculated motor angles to the corresponding motors
  int motor_positions[NUM_MOTORS];
  motor_positions[0] = scaleToMotorRange(motorAngles[2], cw_limits[0], ccw_limits[0]); // Motor ID 1
  motor_positions[1] = scaleToMotorRange(motorAngles[1], cw_limits[1], ccw_limits[1]); // Motor ID 2
  motor_positions[2] = scaleToMotorRange(motorAngles[0], cw_limits[2], ccw_limits[2]); // Motor ID 3
  motor_positions[3] = scaleToMotorRange(motorAngles[3], cw_limits[3], ccw_limits[3]); // Motor ID 4

  for (int i = 0; i < NUM_MOTORS; i++) {
    dxl.setGoalPosition(DXL_IDs[i], motor_positions[i]);
    dxl.setGoalCurrent(DXL_IDs[i], 300);
  }

  if (iters == 0){
    delay(3000); // Delay to allow the motors to reach the desired positions
    calculateErrors();
  }
  
  iters+=1;

// V3
  String posString = "";
  for (int i = 0; i < NUM_MOTORS; i++) {
    int pos = dxl.getPresentPosition(DXL_IDs[i]);
    posString += String(pos);
    if (i < NUM_MOTORS - 1) posString += ",";
  }
  Serial.println(posString);

//  V2 (version that was working but wasn't always printing all angles)
//  for (int i = 0; i < NUM_MOTORS; i++) 
//  {
//    int pos = dxl.getPresentPosition(DXL_IDs[i]);
//    Serial.print(pos);
//    if (i < NUM_MOTORS - 1) {
//      Serial.print(","); // comma separator
//    } else {
//      Serial.println(); // newline at the end
//    }
//  }

//  V1 (Aban)
//  for (int i = 0; i < NUM_MOTORS; i++) 
//  {
//  int pos = dxl.getPresentPosition(DXL_IDs[i]);
//
//  Serial.write(pos & 0xFF);         // LSB
//  Serial.write((pos >> 8) & 0xFF);  // MSB  
//  }
}


int scaleToMotorRange(double angle, int cw_limit, int ccw_limit) {

    // Convert angle in degrees to position units
    int position = angle * 4096 / 360.0;

    // Calculate the center of the motor's range
    int effective_range = abs(ccw_limit - cw_limit);
    int center_position = (cw_limit + ccw_limit)/2;

    if (position > (effective_range/2)){
      Serial.println("Error: Angle exceeds motor's range limits.");
      // Handle the error (e.g., stop the motor, return a safe position, or continue)
      return center_position; // Returning center position as a safe default
    }

    // Offset the position from the zero position (chosen to be the centre of the ranges)
    int scaled_position;

    if (POSITIVE_IS_CCW){
      scaled_position = center_position + position;
    }
    else{
      scaled_position = center_position - position;
    }

    return scaled_position;
}


tuple<int, int> homeMotor(int DXL_ID) {
    const int max_turns = 3 * 4096; // Maximum number of turns for homing in one direction, converted to Dynamixel units
    const int max_current = MAX_CURRENT;  // Maximum current in mA for homing
    const int no_movement_threshold = 10; // Number of loops with no movement to detect a stop
    const int delay_time = 50;    // Delay between checks
    const int movement_threshold = 2;  // Minimum change in position to consider movement
    const float max_position_range = (MAX_POSITION_RANGE * 4096) / 360.0; // Convert degrees to Dynamixel units

    int cw_limit = -1;
    int ccw_limit = -1;
    tuple<int, int> limits;

    Serial.print("Starting homing for motor with ID: ");
    Serial.println(DXL_ID);

    // Disable torque on all other motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (DXL_IDs[i] != DXL_ID) {
            dxl.torqueOff(DXL_IDs[i]);
        }
    }

    // Enable torque for the current motor
    dxl.torqueOn(DXL_ID);

    bool homing_successful = false;

    while (!homing_successful) {
        // Homing in the CW direction with a turn limit (in Dynamixel units)
        Serial.println("Homing in CW direction...");
        dxl.setGoalCurrent(DXL_ID, -max_current, UNIT_MILLI_AMPERE);
        if (detectNoMovementWithinTurns(DXL_ID, delay_time, no_movement_threshold, movement_threshold, max_turns)) {
            cw_limit = int(dxl.getPresentPosition(DXL_ID));  // Normalise within 4096 (motor steps);
            Serial.print("CW Limit found: ");
            Serial.println(cw_limit);

            // Now search for the CCW limit within max_position_range
            Serial.println("Homing in CCW direction...");
            dxl.setGoalCurrent(DXL_ID, max_current, UNIT_MILLI_AMPERE);
            if (detectNoMovementWithinTurns(DXL_ID, delay_time, no_movement_threshold, movement_threshold, max_position_range)) {
                ccw_limit = int(dxl.getPresentPosition(DXL_ID));  // Normalise within 4096 (motor steps);
                Serial.print("CCW Limit found: ");
                Serial.println(ccw_limit);
                homing_successful = true;
            } else {
                Serial.println("No movement detected in CCW direction within position range. Restarting...");
                cw_limit = -1; // Reset the CW limit if the CCW limit isn't found
            }
        } else {
            Serial.println("No movement detected in CW direction within turn limit. Reversing...");

            // Search for the CCW limit first
            Serial.println("Homing in CCW direction...");
            dxl.setGoalCurrent(DXL_ID, max_current, UNIT_MILLI_AMPERE);
            if (detectNoMovementWithinTurns(DXL_ID, delay_time, no_movement_threshold, movement_threshold, max_turns)) {
                ccw_limit = int(dxl.getPresentPosition(DXL_ID));  // Normalise within 4096 (motor steps);
                Serial.print("CCW Limit found: ");
                Serial.println(ccw_limit);

                // Now search for the CW limit within max_position_range
                Serial.println("Homing in CW direction...");
                dxl.setGoalCurrent(DXL_ID, -max_current, UNIT_MILLI_AMPERE);
                if (detectNoMovementWithinTurns(DXL_ID, delay_time, no_movement_threshold, movement_threshold, max_position_range)) {
                    cw_limit = int(dxl.getPresentPosition(DXL_ID));  // Normalise within 4096 (motor steps);
                    Serial.print("CW Limit found: ");
                    Serial.println(cw_limit);
                    homing_successful = true;
                } else {
                    Serial.println("No movement detected in CW direction within position range. Restarting...");
                    ccw_limit = -1; // Reset the CCW limit if the CW limit isn't found
                }
            } else {
                Serial.println("No movement detected in CCW direction within turn limit. Restarting...");
            }
        }
    }

    int offset = -cw_limit; //cw_limit is new zero (if changed, must change following logic)

    cw_limit += offset;
    ccw_limit += offset;
    if (cw_limit < 0 || ccw_limit < 0){
      Serial.println("Error: Offset error, negative positions yielded.");
    }

    Serial.print("Corrected CW Limit: ");
    Serial.println(cw_limit);
    Serial.print("Corrected CCW Limit: ");
    Serial.println(ccw_limit);

    // Disable torque on current motor for setting changes
    dxl.torqueOff(DXL_ID);

    // Set new zero position
    dxl.writeControlTableItem(HOMING_OFFSET, DXL_ID, offset);
    // Set the found limits
    dxl.writeControlTableItem(MIN_POSITION_LIMIT, DXL_ID, cw_limit);
    dxl.writeControlTableItem(MAX_POSITION_LIMIT, DXL_ID, ccw_limit);
    // Prepare for standard movement
    dxl.setOperatingMode(DXL_ID, OP_CURRENT_BASED_POSITION);

    // Re-enable torque on all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        dxl.torqueOn(DXL_IDs[i]);
    }

    Serial.print("Homing completed for motor with ID: ");
    Serial.println(DXL_ID);

    return make_tuple(cw_limit, ccw_limit);
}

bool detectNoMovementWithinTurns(int DXL_ID, int delay_time, int no_movement_threshold, int movement_threshold, int max_position_units) {
  int no_movement_counter = 0;
  int last_position = dxl.getPresentPosition(DXL_ID);
  int initial_position = last_position;

  while (true) {
    delay(delay_time);  // Wait for some time
    int current_position = dxl.getPresentPosition(DXL_ID);

    // Check if the motor is moving
    if (abs(current_position - last_position) < movement_threshold) {
      no_movement_counter++;
      if (no_movement_counter >= no_movement_threshold) {
//        Serial.println("No movement detected, motor has stopped.");
        return true;  // Motor has stopped moving
      }
    } else {
      no_movement_counter = 0;  // Reset if movement is detected
    }

    // Check if the motor has exceeded the maximum number of turns (in position units)
    if (abs(current_position - initial_position) >= max_position_units) {
      Serial.println("Maximum turn limit reached without finding a stop.");
      return false;  // Exceeded turn limit
    }

    last_position = current_position;  // Update last position
  }
}


void processAngles(String data) {
  // Check if any data was actually received
  if (data.length() == 0) {
    return;  // Exit the function if no data was received
  }

  // Split the data into individual angles
  float angles[4];
  int index = 0;
  String angle = "";

  for (int i = 0; i < data.length(); i++) {
    if (data[i] == ',') {
      if (index < 4) {
        angles[index] = angle.toFloat();  // Convert to float
        angle = "";
        index++;
      }
    } else {
      angle += data[i];
    }
  }

  // Convert and store the last angle if we have exactly 4 angles
  if (index < 4) {
    angles[index] = angle.toFloat();
  }

  // Only update the global variables if exactly 4 angles were received
  if (index == 3) {
    theta_R = angles[0];
    theta_W = angles[1];
    theta_G1 = angles[2];
    theta_G2 = angles[3];

    Serial.println("Angles received and processed.");
    Serial.print("theta_R: "); Serial.println(theta_R);
    Serial.print("theta_W: "); Serial.println(theta_W);
    Serial.print("theta_G1: "); Serial.println(theta_G1);
    Serial.print("theta_G2: "); Serial.println(theta_G2);
    iters = 0;
  } else {
    Serial.println("Invalid data received.");
  }
}


void calculateErrors() {
  // Array to hold motor angles (in degrees)
  double motorAngles[4] = {0};

  Serial.println("Motor Angles (in degrees) and Errors:");

  // Calculate motor errors and reverse-engineer motor angles
  for (int i = 0; i < NUM_MOTORS; i++) {
    int goal_position = dxl.readControlTableItem(GOAL_POSITION, DXL_IDs[i]);
    int present_position = dxl.readControlTableItem(PRESENT_POSITION, DXL_IDs[i]);

    // Calculate motor error
    float error = goal_position - present_position;
    error = error / 4096.0 * 360.0;

    // Print motor error
    Serial.print("Error in motor ");
    Serial.print(DXL_IDs[i]);
    Serial.print(": ");
    Serial.print(error);
    Serial.println(" degrees");

    // Reverse-engineer motor angle from position units to degrees
    int midpoint = (cw_limits[i] + ccw_limits[i]) / 2;
    if (POSITIVE_IS_CCW) {
      motorAngles[i] = (present_position - midpoint) / 4096.0 * 360.0;
    } else {
      motorAngles[i] = (midpoint - present_position) / 4096.0 * 360.0;
    }

    // Print motor angles for debugging
    Serial.print("Motor ");
    Serial.print(DXL_IDs[i]);
    Serial.print(": ");
    Serial.print(motorAngles[i]);
    Serial.println(" degrees");
  }

  // Calculate end-effector angles based on the correct motor relationships
  double calculated_theta_R = transmissionMatrix[0][3] * motorAngles[3];
  double calculated_theta_W = transmissionMatrix[1][2] * motorAngles[0];
  double calculated_theta_G1 = transmissionMatrix[2][1] * motorAngles[1] + transmissionMatrix[2][2] * motorAngles[0];
  double calculated_theta_G2 = transmissionMatrix[3][0] * motorAngles[2] + transmissionMatrix[3][2] * motorAngles[0];

  // Output recalculated end-effector angles
  Serial.println("End Effector Angles (in degrees):");
  Serial.print("R (theta_R): ");
  Serial.println(calculated_theta_R);
  Serial.print("W (theta_W): ");
  Serial.println(calculated_theta_W);
  Serial.print("G1 (theta_G1): ");
  Serial.println(calculated_theta_G1);
  Serial.print("G2 (theta_G2): ");
  Serial.println(calculated_theta_G2);

  // Calculate and print errors for end-effector angles
  Serial.print("Error in R (theta_R): ");
  Serial.println(calculated_theta_R - theta_R);
  Serial.print("Error in W (theta_W): ");
  Serial.println(calculated_theta_W - theta_W);
  Serial.print("Error in G1 (theta_G1): ");
  Serial.println(calculated_theta_G1 - theta_G1);
  Serial.print("Error in G2 (theta_G2): ");
  Serial.println(calculated_theta_G2 - theta_G2);
}
