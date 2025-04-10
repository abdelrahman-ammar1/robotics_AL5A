#include "Arduino.h"
#include <Servo.h>
#include <math.h>
#include <BasicLinearAlgebra.h>
#define AL5A
using namespace BLA;
#ifdef AL5A
const float A = 3.75; // Shoulder segment length
const float B = 4.25; // Elbow segment length
#endif
// Structured poses
struct Pose {
  float x;
  float y;
  float z;
};

// Finalized hardcoded poses
Pose initPose  = {1.50, 3.75, 140.00};
Pose pickPose  = {3.25, 1.25,  90.00};
Pose placePose = {2.25, 4.75, 177.50};

// RRT Stuff -----------------------------------------------------------------------------------------
// Define robot parameters
const int NUM_JOINTS = 4;
const float JOINT_LIMIT_MIN[NUM_JOINTS] = {-1.570796, 0, 0, 0};
const float JOINT_LIMIT_MAX[NUM_JOINTS] = {1.570796, 3.14159, 3.14159, 3.14159};

const int MAX_ITERATIONS = 400;
const float STEP_SIZE = 0.2;
const float GOAL_THRESHOLD = 0.2;
const int MAX_NODES = MAX_ITERATIONS / 2;

// Structure to represent a node in the RRT tree
struct Node {
    Matrix<NUM_JOINTS, 1> config;
    Node* parent;
};

// Function for basic random number generation
float randomFloat(float min, float max) {
    return min + (max - min) * (float)random(10000) / 10000.0;
}

// Function to normalize a vector
Matrix<NUM_JOINTS, 1> Normalize(const Matrix<NUM_JOINTS, 1>& vec) {
    float norm = 0.0;
    for (int i = 0; i < NUM_JOINTS; i++) {
        norm += vec(i) * vec(i);
    }
    norm = sqrt(norm);
    if (norm > 0) {
        return vec / norm;
    } else {
        return vec;
    }
}

// Function to compute the norm of a vector
float Norm(const Matrix<NUM_JOINTS, 1>& vec) {
    float norm = 0.0;
    for (int i = 0; i < NUM_JOINTS; i++) {
        norm += vec(i) * vec(i);
    }
    return sqrt(norm);
}

// Function to generate a random configuration and includes a goal biasing
Matrix<NUM_JOINTS, 1> getRandomConfig(const Matrix<NUM_JOINTS, 1>& theta_goal) {
    float goal_bias = 0.2; // 10% chance to select the goal directly

    if (randomFloat(0, 1) < goal_bias) {
        //Serial.println("Biasing towards goal.");
        return theta_goal;
    }

    Matrix<NUM_JOINTS, 1> config;
    for (int i = 0; i < NUM_JOINTS; i++) {
        config(i) = randomFloat(JOINT_LIMIT_MIN[i], JOINT_LIMIT_MAX[i]);
    }
    return config;
}

// Function to check joint limits
bool checkJointLimits(const Matrix<NUM_JOINTS, 1>& config) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (config(i) < JOINT_LIMIT_MIN[i] || config(i) > JOINT_LIMIT_MAX[i]) {
            return false;
        }
    }
    return true;
}

// Function to extend the tree by moving from the nearest node towards the target configuration
bool extendTree(const Matrix<NUM_JOINTS, 1>& nearest_config, const Matrix<NUM_JOINTS, 1>& target_config, float step_size, Matrix<NUM_JOINTS, 1>& new_config) {
    // Compute the unit direction vector from the nearest configuration towards the target
    Matrix<NUM_JOINTS, 1> direction = Normalize(target_config - nearest_config);
    
    // Generate a new configuration by moving step_size along the computed direction
    new_config = nearest_config + step_size * direction;

    // Check if the new configuration is valid (within joint limits)
    if (!checkJointLimits(new_config)){
        //Serial.println("Out of bounds");
        return false;
    }
    else {
    //Serial.println("Node added to tree.");
    }
    return true;
}

// Rapidly-exploring Random Tree (RRT) algorithm for path planning
// This function attempts to find a path from theta_init to theta_goal using an exploration tree
Matrix<NUM_JOINTS, 1>* rrt(const Matrix<NUM_JOINTS, 1>& theta_init, const Matrix<NUM_JOINTS, 1>& theta_goal, int& path_length) {
    Node** nodes = new Node*[MAX_NODES]; // Dynamic array of node pointers
    int node_count = 0; // Keeps track of the number of nodes in the tree

    // Create and initialize the root node (starting configuration)
    Node* root = new Node;
    root->config = theta_init;
    root->parent = nullptr;
    nodes[node_count++] = root;

    // Main RRT exploration loop
    for (int k = 0; k < MAX_ITERATIONS; k++) {
        if (k%10==0){
        Serial.print("Iteration: ");
        Serial.println(k);
        }
        //Serial.print("Nodes in tree: ");
        //Serial.println(node_count);

        // TODO: Generate a random configuration within joint limits
        Matrix<NUM_JOINTS, 1> rand_config = getRandomConfig(theta_goal);

        // TODO: Find the nearest node in the tree to the randomly generated configuration
        int nearest_index = 0;
        float min_dist = Norm(nodes[0]->config - rand_config);

        for (int i =1; i < node_count; i++) {
            float dist = Norm(nodes[i]->config - rand_config);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_index = i;
            }
        }
  if(k%10==0){
    Serial.print("Nearest Node Distance: ");
        Serial.println(min_dist);
  }


        Matrix<NUM_JOINTS, 1> new_config;

        // Try to extend the tree from the nearest node towards the random configuration
        if (extendTree(nodes[nearest_index]->config, rand_config, STEP_SIZE, new_config)) {
            // TODO: Create a new node and assign it to the new configuration
            Node* new_node = new Node;
            new_node->config = new_config;
            //
            new_node->parent = nodes[nearest_index];

            // Check if the new node is close enough to the goal (successful path found)
            if (Norm(new_node->config - theta_goal) < GOAL_THRESHOLD) {
                // TODO: Determine path length first
                int current_path_length = 0;
                Node* current = new_node;
                while (current != nullptr) {
                  current_path_length++;
                  current = current->parent;
                }

                // Allocate memory for the path and store configurations in reverse order
                Matrix<NUM_JOINTS, 1>* path_array = new Matrix<NUM_JOINTS, 1>[current_path_length];
                path_length = current_path_length;
                
                current = new_node;
                for (int i = path_length - 1; i >= 0; i--) {
                    path_array[i] = current->config;
                    current = current->parent;
                }

                // Free memory for the created tree nodes before returning the path
                for (int i = 0; i < node_count; i++) {
                    delete nodes[i];
                }
                delete[] nodes;

                return path_array; // Return the computed path
            }

            // Add the new node to the tree if space allows
            if (node_count < MAX_NODES) {
                nodes[node_count++] = new_node;
            } else {
                delete new_node;
            }
        }
    }

    // If no valid path is found, clean up allocated nodes
    for (int i = 0; i < node_count; i++) {
        delete nodes[i];
    }
    delete[] nodes; // Free the node array memory

    return nullptr; // No valid path found
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Servo pin assignments
#define Base_pin 2
#define Shoulder_pin 3
#define Elbow_pin 4
#define Wrist_pin 10
#define Gripper_pin 11
#define WristR_pin 12

// Servo objects
Servo Elb, Shldr, Wrist, Base, Gripper, WristR;

// Arm current position initialization
float X = 4, Y = 4, Z = 90;
int G = 90, WR = 90;
float WA = 0;

// Temporary position variables (controlled via keyboard)
float tmpx = 4, tmpy = 4, tmpz = 90;
int tmpg = 90, tmpwr = 90;
float tmpwa = 0;


void goToPose(Pose pose, float wa, int g) {
  Arm(pose.x, pose.y, pose.z, g, wa, 90); // 90 = WR (ignored)
}

int Arm(float x, float y, float z, int g, float wa, int wr) {
  float M = sqrt((y * y) + (x * x));
  //Serial.print("M: "); Serial.println(M);
  if (M <= 0) {
    //Serial.println("IK Error: M <= 0");
    return 1;
  }

  float A1 = atan2(y, x);  // use atan2 to avoid x <= 0 rejection
  float A2 = acos((A * A - B * B + M * M) / (2 * A * M));
  float Elbow = acos((A * A + B * B - M * M) / (2 * A * B));
  float Shoulder = A1 + A2;

  float ElbowDeg = Elbow * 57.295779;
  float ShoulderDeg = Shoulder * 57.295779;

  if ((int)ElbowDeg <= 0 || (int)ShoulderDeg <= 0) {
    //Serial.println("IK Error: Elbow or Shoulder <= 0");
    return 1;
  }

  float Wris = abs(wa - ElbowDeg - ShoulderDeg) - 90;

  Elb.write(180 - ElbowDeg);
  Shldr.write(ShoulderDeg);
  Wrist.write(180 - Wris);
  Base.write(z);
  WristR.write(wr);
  Gripper.write(g);

  return 0;
}

void moveToJointAngles(float base_deg, float shoulder_deg, float elbow_deg, float wrist_deg, int gripper, int wristR) {
  Base.write(base_deg);
  Shldr.write(shoulder_deg);
  Elb.write(180 - elbow_deg);  // Consistent with existing Arm() behavior
  Wrist.write(180 - wrist_deg);
  WristR.write(wristR);
  Gripper.write(gripper);
}

bool computeIK(Pose pose, float wa, Matrix<NUM_JOINTS, 1>& theta_out) {
  float M = sqrt((pose.y * pose.y) + (pose.x * pose.x));
  if (M <= 0 || pose.x <= 0) return false;

  float A1 = atan2(pose.y, pose.x);
  float A2 = acos((A * A - B * B + M * M) / (2 * A * M));
  float Elbow = acos((A * A + B * B - M * M) / (2 * A * B));
  float Shoulder = A1 + A2;

  float ElbowDeg = Elbow * 57.295779;
  float ShoulderDeg = Shoulder * 57.295779;
  float WristDeg = abs(wa - ElbowDeg - ShoulderDeg) - 90;

  if ((int)ElbowDeg <= 0 || (int)ShoulderDeg <= 0) return false;

  theta_out(0) = radians(pose.z - 90);     // Base (centered)
  theta_out(1) = radians(ShoulderDeg);
  theta_out(2) = radians(ElbowDeg);
  theta_out(3) = radians(WristDeg);

  return true;
}

void executeRRTPath(Matrix<NUM_JOINTS, 1> from, Matrix<NUM_JOINTS, 1> to, int g, float wa) {
  int path_length = 0;
  Matrix<NUM_JOINTS, 1>* path = rrt(from, to, path_length);
  
  if (path == nullptr) {
    //Serial.println("RRT failed to find a path.");
    return;
  }

  //Serial.print("Executing RRT path of length: ");
  //Serial.println(path_length);

  for (int i = 0; i < path_length; i++) {
    Matrix<NUM_JOINTS, 1> config = path[i];

    float base_deg = degrees(config(0)) + 90;
    float shoulder_deg = degrees(config(1));
    float elbow_deg = degrees(config(2));
    float wrist_deg = degrees(config(3));

    moveToJointAngles(base_deg, shoulder_deg, elbow_deg, wrist_deg, g, 90);  // 90 = WR
    delay(300); 
  }

  delete[] path;
}

void executeFinalSequence() {
  Matrix<NUM_JOINTS, 1> theta_pick, theta_place;

  // === Move to Init first
  goToPose(initPose, 40, 90); // Mid wrist, neutral gripper
  delay(1000);

  // === Compute joint angles
  if (!computeIK(pickPose, 0, theta_pick)) {
    return;
  }
  if (!computeIK(placePose, 80, theta_place)) {
    Serial.println("Failed IK: Place");
    return;
  }

  // === Init → Pick (direct)
  Gripper.write(30);  // Open gripper
  delay(300);
  goToPose(pickPose, 0, 30);  // Wrist down, gripper open
  delay(2000);

  // === Grab
  Gripper.write(120); // Close gripper
  delay(1000);

  // === Pick → Place (RRT)
  executeRRTPath(theta_pick, theta_place, 120, 80); // Wrist up, gripper closed
  delay(1000);

  // === Drop
  Gripper.write(30); // Open gripper
  delay(1000);

  // === Return to Init (direct)
  goToPose(initPose, 40, 90); // Wrist neutral, gripper open
  delay(500);

  Serial.println("Sequence complete.");
}

//-----------------------------------------------------
// Setup and Loop
//-----------------------------------------------------
void setup() {
  Serial.begin(9600);
  Base.attach(Base_pin);
  Shldr.attach(Shoulder_pin);
  Elb.attach(Elbow_pin);
  Wrist.attach(Wrist_pin);
  Gripper.attach(Gripper_pin);
  WristR.attach(WristR_pin);
  
  Arm(tmpx, tmpy, tmpz, tmpg, tmpwa, tmpwr);
  Serial.println(F("Press 'p' to execute sequence."));
}

void loop() {
  if (Serial.available() > 0) {
    char action = Serial.read();
    switch(action) {
      case 'p':
        executeFinalSequence();
        break;
    }
    delay(100);
  }
}
