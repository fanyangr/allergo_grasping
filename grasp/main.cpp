// the code is coded by Fan in Aug 2019,
// adapted from the Allegro hand control code by Mikael
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>  //_getch
#include <pthread.h>
#include "canAPI.h"
#include "rDeviceAllegroHandCANDef.h"
#include <BHand/BHand.h>
#include "Sai2Model.h"
#include <signal.h>
#include <vector>
#include <deque>
#include <unistd.h>
#include <fstream>
#include "CDatabaseRedisClient.hpp"
#include <iostream>

using namespace std;
using namespace Eigen;
// ============= For REDIS =================

// redis keys
static const string ALLGERO_COMMAND = "allegro::command";
const string PYTHON_START_FLAG_KEY = "start_flag_key";
const string CONTACT_POSITION_1_KEY = "contact_position_1_key";
const string CONTACT_POSITION_2_KEY = "contact_position_2_key";
const string CONTACT_POSITION_3_KEY = "contact_position_3_key";
const string CONTACT_NORMAL_1_KEY = "contact_normal_1_key";
const string CONTACT_NORMAL_2_KEY = "contact_normal_2_key";
const string CONTACT_NORMAL_3_KEY = "contact_normal_3_key";
const string FORCE_1_KEY = "force_1_key";
const string FORCE_2_KEY = "force_2_key";
const string FORCE_3_KEY = "force_3_key";
const string MASS_KEY = "mass_key";
const string COM_KEY = "com_key";
const string SCORE_KEY = "score_key";
const string JOINT_ANGLE_KEY = "joint_angle_key";
const string THUMB_FINGRE_TIP_KEY = "thumb_finger_tip_key";
const string THUMB_DESIRED_POSITION_KEY = "thumb_desired_position_key";


/** Global constants for REDIS host and port. */
static const string REDIS_HOST = "127.0.0.1";
static const int REDIS_PORT = 6379;


/** Global REDIS interface variables */
redisContext *GLOBAL_Redis_Context;
redisReply *GLOBAL_Redis_Reply;

bool initializeRedis()
{
  GLOBAL_Redis_Reply = NULL;
  GLOBAL_Redis_Context = redisConnect(REDIS_HOST.c_str(), REDIS_PORT);
  if (GLOBAL_Redis_Context->err) {
    std::cerr << "Error: " <<  GLOBAL_Redis_Context->errstr << std::endl;
    return false;
  } else {
    std::cout << "REDIS Connection Successful.\n" << std::endl;
  redisCommand(GLOBAL_Redis_Context, "SET %s o", ALLGERO_COMMAND.c_str());
    return true;
  }
}


char getCommandFromRedis()
{	
    GLOBAL_Redis_Reply = (redisReply *) redisCommand(GLOBAL_Redis_Context,
        "GET %s", ALLGERO_COMMAND.c_str());

    char buf = 0;
    sscanf(GLOBAL_Redis_Reply->str, "%c", &buf);
    freeReplyObject(GLOBAL_Redis_Reply);
	// printf("%c\n",buf);
    return buf;
}

// =========================================

typedef char    TCHAR;
#define _T(X)   X
#define _tcsicmp(x, y)   strcmp(x, y)

/////////////////////////////////////////////////////////////////////////////////////////
// for CAN communication
const double delT = 0.003;
int CAN_Ch = 0;
bool ioThreadRun = false;
pthread_t        hThread;
pthread_t        cThread;  // control thread
int recvNum = 0;
int sendNum = 0;
double statTime = -1.0;
AllegroHand_DeviceMemory_t vars;

double curTime = 0.0;

/////////////////////////////////////////////////////////////////////////////////////////
// for custom pd controller
bool custom_PD = false;
double q_prev[MAX_DOF];
double dq[MAX_DOF];
double dq_filter_input[MAX_DOF];
double dq_prev_filter_input[MAX_DOF];
double dq_prev_prev_filter_input[MAX_DOF];
double dq_filtered[MAX_DOF];
double dq_prev_filtered[MAX_DOF];
double dq_prev_prev_filtered[MAX_DOF];

// pregrasp for the box
 double origin_config[] =
{
  10.0 / 180.0 * M_PI, 0.7, 0.7, 0.5,
  0.0, 0.7, 0.7, 0.5,
  -15.0 / 180.0 * M_PI, 0.5, 0.7, 0.5,
  1.57, 0.0, -0.2, 0.7
};
/* double origin_config[] =
{
  10.0 / 180.0 * M_PI, 0.8, 0.7, 0.5,
  0.0, 0.8, 0.7, 0.5,
  -15.0 / 180.0 * M_PI, 0.5, 0.7, 0.5,
  1.57, 0.0, -0.2, 0.7
};*/

// ==================Several flags in the control loop===============
// the flag which marks whether we should use dynamic consistence (hierachical controller)
//  to maintain the original configurations. It's only used in the operational space position control function
// TODO: 
// but it's not reliable, since I block the palm torque in this algorithmn. If we set the coefficient 
// pretty small, then it does help to avoid singularities, but if we set it too large, it's unstabe. 
// it's better to set it to be false now, but our in our last successful trial it's true, so I just leave it true
// and the trembling problem in position control may come from this part.
const bool dynamic_consistence_flag = true; 
// since this algorithm is also using hierachical controller, it's not reliable, better to set it false
const bool joint_limit_avoidance_flag = false;
const double avoidance_coefficient = 20;
// the flag indicating whether to compensate the friction. compensation is nothing but adding some predefined torque
// to the command torque basing on the sign of the command torque
const bool friction_compensation_flag = true;
// whether to use predefined normal (1,0,0) (-1,0,0) (-1,0,0). it's only useful when we want to debug the box
const bool predefined_normal_flag = false;

// ===================Filter============================================
// filter
double gain = 2.419823131e+01;                              // 25 Hz
double filter_coeffs[] = {-0.5136414053, 1.3483400678};
// double gain = 4.020427297e+00;                                 // 75 Hz
// double filter_coeffs[] = {-0.1774700802, 0.1825509574};

// ===================Driver initialization ============================
// for BHand library
BHand* pBHand = NULL;
// joint angle
double q[MAX_DOF]; 
double q_des[MAX_DOF];  // this variable may not be useful here
double gravity_tourque[MAX_DOF];
double control_torque[MAX_DOF];
double tau_des[MAX_DOF];
double cur_des[MAX_DOF];

// USER HAND CONFIGURATION
const bool	RIGHT_HAND = true;
const int	HAND_VERSION = 3;

const double tau_cov_const_v2 = 800.0; // 800.0 for SAH020xxxxx
const double tau_cov_const_v3 = 1200.0; // 1200.0 for SAH030xxxxx


const double enc_dir[MAX_DOF] = { // SAH030xxxxx
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0
};
const double motor_dir[MAX_DOF] = { // SAH030xxxxx
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0
};

const int enc_offset[MAX_DOF] = { // SAH030C033R
  -1591, -277, 545, 168,
  -904, 53, -233, -1476,
  2, -987, -230, -106,
  -1203, 361, 327, 565
};
// calibrating parameters for the joint angles 
const double q_offset[MAX_DOF] = 
{
  -15.0 / 180.0 * 3.1415,0.0133145,0.00284042,0,
  -10.0 / 180.0 * 3.1415,0.0139358,-0.00284042,-7.0 / 180.0 * 3.1415926,
  -0.02512,-0.0126044,-0.00399434,0.00301795,
  -6.0 / 180.0 * 3.1415926, 15.0 / 180.0 * 3.1415926, 0.0, -0.0157111,
};

// ==============================Function declarations========================================
char Getch();
void PrintInstruction();
void MainLoop();
bool OpenCAN();
void CloseCAN();
int GetCANChannelIndex(const TCHAR* cname);
bool CreateBHandAlgorithm();
void DestroyBHandAlgorithm();
void ComputeTorqueCustom();
void ComputeGravityTorque();
// convert the angle representation from driver to Sai2
VectorXd driver_to_sai2 (double q[MAX_DOF]);
// convert the angle representation from Sai2 to driver, q is the array where the result should be stored
void sai2_to_driver(VectorXd _q, double q[MAX_DOF]);
// the function used in the finger position control command
// just a pd controller with kv = sqrt(kp) / 3
VectorXd compute_position_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position, double kp);
// a pid controller
VectorXd compute_position_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position, double kp, Vector3d& i_error);
// a pid controlle with adjustable parameters
VectorXd compute_position_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position, double kp, Vector3d& i_error, double ki, double kv);
// velocity saturation, which is not used in the code 
VectorXd compute_velocity_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position, double kp, double kv, double desired_velocity);
// pd controller to compute the torque to get back to the specific configuration
VectorXd compute_joint_cmd_torques(Sai2Model::Sai2Model* robot, VectorXd desired_joint_angles);
// pd controller to compute the torque to get back to the specific configuration for only a specific finger
VectorXd compute_joint_cmd_torques_one_finger(Sai2Model::Sai2Model* robot, VectorXd desired_joint_angles, int index);
// when applying the desired force, we add a position control on the directions which are vertical to the desired force
VectorXd bounce_back(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d optimal_position, Vector3d optimal_forces, double kp, double kv);
// set any vectors of Vector3d to be 0
void set_zero(vector<Vector3d>& variables);
// compute the actual contact points base on the origin of the hemisphere and the normals, it's just a displacement
vector<Vector3d> hemisphere_contact_points(vector<Vector3d> contact_points, vector<Vector3d> normals);
// the function used in the finger force control, used to achieve compliance
// input desired position and the desired maginitude of force
VectorXd compute_force_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position, double force_requeired);
// input the desired vector of force
VectorXd compute_force_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_force);
// this function is used to detect surface normal by sampling several points in the vicinity
// It can only be called when the finger tip is making contact with the object surface
// returns the torque needed 
VectorXd detect_surface_normal(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d original_pos, int& state, deque<double>& velocity_record, vector<Vector3d>& contact_points, Vector3d& normal, int& static_counter, Vector3d& i_error, double kp);
// the function used to make contact with the object
VectorXd make_contact(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position, double force_requeired, int& contact_flag, int& static_counter, deque<double>& velocity_record);
// this function is used to check whether we can only 2 finger to have a leagal grasp
bool check_2_finger_grasp(vector<Vector3d> contact_points,vector<Vector3d> normals, double friction_coefficient);
bool check_3_finger_grasp(vector<Vector3d> contact_points,vector<Vector3d> normals, double friction_coefficient);
// we compute the torques based on the each finger command, then we will need to block the torques before we add them together
vector<VectorXd> block_torque(vector<VectorXd> temp_torques);
// send the desirede information to redis
void send_to_redis(CDatabaseRedisClient& redis_cli, vector<Vector3d> contact_points, vector<Vector3d> normals);


/////////////////////////////////////////////////////////////////////////////////////////
// Read keyboard input (one char) from stdin
char Getch()
{
  /*#include <unistd.h>   //_getch*/
  /*#include <termios.h>  //_getch*/
  char buf=0;
  struct termios old={0};
  fflush(stdout);
  if(tcgetattr(0, &old)<0)
    perror("tcsetattr()");
  old.c_lflag&=~ICANON;
  old.c_lflag&=~ECHO;
  old.c_cc[VMIN]=1;
  old.c_cc[VTIME]=0;
  if(tcsetattr(0, TCSANOW, &old)<0)
    perror("tcsetattr ICANON");
  if(read(0,&buf,1)<0)
    perror("read()");
  old.c_lflag|=ICANON;
  old.c_lflag|=ECHO;
  if(tcsetattr(0, TCSADRAIN, &old)<0)
    perror ("tcsetattr ~ICANON");
  printf("%c\n",buf);
  return buf;
}

/////////////////////////////////////////////////////////////////////////////////////////
// For code from Sai2
bool runloop = false;
void sighandler(int sig)
{ runloop = false; }

const string robot_file = "hand.urdf";
const string robot_name = "Hand3Finger";

//=========================predefined variables===================================
#define NUM_OF_FINGERS_IN_MODEL 4
#define NUM_OF_FINGERS_USED     3
#define ROBOT_DOF               22

// if we want to detect velocity change to detect contact, the coefficient of v / v_prev
#define CONTACT_COEFFICIENT     0.0 
// if we detect velocity change to detect contact, the minimum contact velocity
#define MIN_COLLISION_V         0.001
#define FRICTION_COEFFICIENT    0.5
// if the velocity is smaller than it, then it means static
#define STATIC_THRESHOLD        0.005
// the tolerance for positioning a finger
#define POSITION_ERROR_TOLER    0.01    
double surface_force = 0.3; 
double prob_distance = 0.01; // how much you want the to prob laterally in normal detection step 
double displacement_dis = 0.01;  // how much you wanna move awat from the original point in normal detection step


// ==============================State machine================================
#define PRE_GRASP               0
#define FINGER_MOVE_CLOSE       1
#define DETECT_NORMAL_THUMB     2
#define DETECT_NORMAL_OTHER     3
#define OPTIMIZE                4
#define RE_PROB                 5
#define CHECK                   6
#define POSITION_FINGERS        7
#define APPLY_FORCE             8
#define LIFT                    9
#define TEST                    9999

int state = PRE_GRASP;
// the counter used to delay the state transistion, which is better than sleep function
int delay_counter = 0;
// the initial guess of the CoM of object in the robor frame, and it will be updated when the robor has made the contact
Vector3d CoM_of_object = Vector3d(0.03, 0.03, - 0.1);

double mass = 5.0 * 0.160;
// radius of the hemisphere
double radius = 0.014;

// ----------------------------------------------------------------------------------------------

static void* sai2 (void * inst)
{
  // initialize redis
  // ===================================================
  HiredisServerInfo server;
  char hostname [] = "127.0.0.1";
  server.hostname_ = hostname;
  server.port_ = 6379;
  server.timeout_.tv_sec = 1;
  server.timeout_.tv_usec = 500000;
  CDatabaseRedisClient redis_cli;
  redis_cli.serverIs(server);
  // ===============================================
  
  // set up signal handler
  signal(SIGABRT, &sighandler);
  signal(SIGTERM, &sighandler);
  signal(SIGINT, &sighandler); 

  // robot model
  // ==============================================
  Sai2Model::Sai2Model* robot = new Sai2Model::Sai2Model(robot_file, false);

  robot->_q = driver_to_sai2(q);
  robot->_dq = driver_to_sai2(dq);
  robot->updateModel();
  int dof = robot->dof();
  VectorXd friction_compensation_pos = VectorXd::Zero(dof);
  friction_compensation_pos << 0,0,0,0,0,0,
  0.0,0.0,0.0,0.02,
  0,0.05,0.03,0.001,
  0,0.05,0.03,0.001,
  0,0.02,0.01,0.002;
  // ========================================
  // initialization of several parameters
  // the gravity compensation torques we add to the original gravity compensation from Bhand library
  VectorXd gravity;

  VectorXd pregrasp_angles = driver_to_sai2(origin_config);
  bool python_start_flag = false;
  redis_cli.setCommandIs(PYTHON_START_FLAG_KEY, to_string(python_start_flag));
  ofstream output;
  // record the positions of the current fingers
  vector<Vector3d> current_finger_position;
  // the command torques sent to the driver
  // ------------torque command--------------
  VectorXd command_torques = VectorXd::Zero(dof);
  // the command torques computed from each finger, if we add it up, we will get the command torque
  vector<VectorXd> finger_command_torques;

  MatrixXd N_prec = MatrixXd::Identity(dof,dof);
  for (int i = 0; i < NUM_OF_FINGERS_IN_MODEL; i++)
  {
    finger_command_torques.push_back(VectorXd::Zero(dof));
  }
  // the finger command torques before blocking it
  vector<VectorXd> temp_finger_command_torques = finger_command_torques; // the raw command torques before blocking
  vector<VectorXd> bounce_back_torques = finger_command_torques;
  // the joint torques computed from the joint command
  VectorXd joint_torques = VectorXd::Zero(dof);

  // ----------------------------------------------------
  //link names and postures
  vector<string> link_names;
  vector<Affine3d> poses;
  Affine3d identity_pose = Affine3d::Identity();
  Affine3d temp_pose = Affine3d::Identity();
  temp_pose.translation() = Vector3d(0.048,0.0,0.0);
  poses.push_back(temp_pose);
  temp_pose.translation() = Vector3d(0.0305, 0.0, 0.0); //0.0305
  poses.push_back(temp_pose);
  poses.push_back(temp_pose);
  poses.push_back(temp_pose);

  link_names.push_back("finger0-link3");
  link_names.push_back("finger1-link3");
  link_names.push_back("finger2-link3");
  link_names.push_back("finger3-link3");
  // ----------------------------------------------------
  // the variables may be used in detecting the surface normals

  // the vector used to record the velocity in the finger move close state
  vector< deque<double> > velocity_record;
  vector< deque<double> > detect_velocity_record;
  vector<int> detect_states;
  vector< vector<Vector3d> > contact_points;
  vector<Vector3d> normals;
  vector<int> finger_contact_flag; // finger0, 1, 2, 3
  vector<int> static_counter;

  // -----------------------------------------------
  
  vector<Vector3d> i_errors;
  Vector3d current_centroid = Vector3d::Zero();
  vector<Vector3d> optimal_positions;
  vector<Vector3d> optimal_forces;
  
  // -----------------------------------------------
  // set to zero
  for(int i = 0; i < NUM_OF_FINGERS_USED; i++)
  {
    deque<double> temp_queue;
    temp_queue.push_back(0.0);
    temp_queue.push_back(0.0);
    velocity_record.push_back(temp_queue);
    detect_velocity_record.push_back(temp_queue);

    current_finger_position.push_back(Vector3d::Zero());

    detect_states.push_back(0);

    normals.push_back(Vector3d::Zero());

    finger_contact_flag.push_back(0);

    static_counter.push_back(0);

    i_errors.push_back(Vector3d::Zero());

    optimal_positions.push_back(Vector3d::Zero());
    optimal_forces.push_back(Vector3d::Zero());

  }

  int reprob_times = 0; // the times the thumb reprobs in the optimization process.
  double min_score_threshold = mass * 9.87;
  double max_score_threshold = mass * 9.87 * 5; 

  runloop = true ;
  long long int loop_counter = 0;


  while(runloop)
  {
    robot->_q = driver_to_sai2(q);
    robot->_dq = driver_to_sai2(dq_filtered);
    Vector3d thum_finger_tip_pos = Vector3d::Zero();
    robot->position(thum_finger_tip_pos, link_names[0], poses[0].translation());
    redis_cli.setEigenMatrixDerived(THUMB_FINGRE_TIP_KEY, thum_finger_tip_pos);
    
    robot->updateModel();
    redis_cli.setEigenMatrixDerived(JOINT_ANGLE_KEY, robot->_q);
    robot->gravityVector(gravity);
    // the gravity compensation is too much for these joints, so devide them by 2.
    gravity[10] /= 2;
    gravity[14] /= 2;
    gravity[18] /= 2;


    if (state == PRE_GRASP)
    {
      joint_torques = compute_joint_cmd_torques(robot, pregrasp_angles);
      /* //the operational space controller
        temp_finger_command_torques[0] = compute_position_cmd_torques(robot, link_names[0], poses[0].translation(), Vector3d(-0.03, 0.02, -0.1), 100.0, i_errors[0]);
        temp_finger_command_torques[1] = compute_position_cmd_torques(robot, link_names[1], poses[1].translation(), Vector3d(0.1, 0.055, -0.12), 100.0, i_errors[1]);
        temp_finger_command_torques[2] = compute_position_cmd_torques(robot, link_names[2], poses[2].translation(), Vector3d(0.1, 0.0, -0.12), 100.0, i_errors[2]);
        temp_finger_command_torques[3] = compute_position_cmd_torques(robot, link_names[3], poses[3].translation(), Vector3d(0.1, -0.065, -0.07), 100.0);
        finger_command_torques = block_torque(temp_finger_command_torques);
        if (palm_command_torques.norm() + finger_command_torques[0].norm() + finger_command_torques[1].norm() + finger_command_torques[2].norm() < 0.65)
      */            
      if (joint_torques.norm() < 0.7)
      {
        delay_counter++;
        if (delay_counter > 20000)
        {
          state = FINGER_MOVE_CLOSE;
          //sleep(1.5); //  a necessary pause to clean off the memory and buffer the mechanism
          set_zero(i_errors);    
          cout << " the robot start moving the finger to make contact" << endl << endl;
          delay_counter = 0 ;
        }
      }
    }

    else if (state == FINGER_MOVE_CLOSE)
    { 
      for(int i = 0; i < NUM_OF_FINGERS_USED; i++)
      {
        // if the finger hasn't made contact, then make contact, if it has, then maintain its position
        if (finger_contact_flag[i] == 0)
        {
          temp_finger_command_torques[i] = make_contact(robot, link_names[i], poses[i].translation(), CoM_of_object, surface_force, finger_contact_flag[i], static_counter[i], velocity_record[i]);
          if (finger_contact_flag[i] == 1)  
          {            
            robot->position(current_finger_position[i], link_names[i], poses[i].translation());
          }
        }
        // maintain the current position after contact
        else if (finger_contact_flag[i] == 1)
        {
          temp_finger_command_torques[i] = compute_position_cmd_torques(robot, link_names[i], poses[i].translation(), current_finger_position[i], 65.0);
            // finger_command_torques[i].block(6 + 4 * i ,0 ,4, 1) = temp_finger_command_torques[i].block(6 + 4 * i, 0 ,4 ,1 );
        } 
      }

      // keep the position of fingers that are not used
      for (int j = NUM_OF_FINGERS_USED; j < NUM_OF_FINGERS_IN_MODEL; j++)
      {
        temp_finger_command_torques[j] = compute_position_cmd_torques(robot, link_names[j], poses[j].translation(), Vector3d(0.1, -0.075, -0.06), 65.0);
        // finger_command_torques[j].block(6+4*j,0,4,1) = temp_finger_command_torques[j].block(6+4*j,0,4,1);
      }

      finger_command_torques = block_torque(temp_finger_command_torques);

      int sum_of_contact = 0;
      for (int j = 0; j < NUM_OF_FINGERS_USED; j++)
      {
        sum_of_contact += finger_contact_flag[j];
      }
      if (sum_of_contact == NUM_OF_FINGERS_USED)
      {
        delay_counter++;
        if (delay_counter > 40000)
        {
          CoM_of_object = (2 * current_finger_position[0] + current_finger_position[1] + current_finger_position[2]) / 4;
          cout << "This is the position of the CoM: " << endl;
          cout << CoM_of_object << endl;
          state = DETECT_NORMAL_OTHER;
          // the position of fingre 1 and 2 now will be used in the final grasp
          optimal_positions[1] = current_finger_position[1];
          optimal_positions[2] = current_finger_position[2];
          cout << " the robot start detecting the surface normal" << endl << endl;
          for (int j = 0; j < NUM_OF_FINGERS_USED; j++)
          {
            vector< Vector3d > temp_vector;
            temp_vector.push_back(current_finger_position[j]);
            contact_points.push_back(temp_vector);
          }
          delay_counter = 0 ;
        }
      }
    }

    else if (state == DETECT_NORMAL_OTHER)
    {
      double sum_of_normal = 0.0;
      for (int i = 1; i < NUM_OF_FINGERS_USED; i++)
      {
        temp_finger_command_torques[i] = detect_surface_normal(robot, link_names[i], poses[i].translation(), current_finger_position[i], detect_states[i], detect_velocity_record[i], contact_points[i], normals[i], static_counter[i], i_errors[i], 65.0);
        // finger_command_torques[i].block(6 + 4 * i ,0 ,4, 1) = temp_finger_command_torques[i].block(6 + 4 * i, 0 ,4 ,1 );
        sum_of_normal += normals[i].norm();

        // the thumb command
        temp_finger_command_torques[0] = compute_position_cmd_torques(robot, link_names[0], poses[0].translation(),current_finger_position[0], 65.0, i_errors[0]);

      for (int j = NUM_OF_FINGERS_USED; j < NUM_OF_FINGERS_IN_MODEL; j++)
      {
        temp_finger_command_torques[j] = compute_position_cmd_torques(robot, link_names[j], poses[j].translation(), Vector3d(0.1, -0.065, -0.07), 65.0);
        // finger_command_torques[j].block(6 + 4 * j, 0, 4, 1) = temp_finger_command_torques[j].block(6 + 4 * j,0,4,1);
      }
      finger_command_torques = block_torque(temp_finger_command_torques);

      if(sum_of_normal > (double(NUM_OF_FINGERS_USED) - 1 - 0.5))
      {
        cout << "all the other normals detected" << endl;
        if(predefined_normal_flag == true)
        {
          normals[1] = Vector3d(-1.0, 0.0, 0.0);
          normals[2] = Vector3d(-1.0, 0.0, 0.0); 
        }
        // this part is to write the results into a txt file, and we have a python code to visualise the result.
        output.open("output.txt");
        for ( int i = 1; i < NUM_OF_FINGERS_USED; i++)
        {
          for ( int j = 0; j < contact_points[i].size() ; j++)
          {
            for (int k = 0; k < 3; k++)
            {
              output << contact_points[i][j][k]<<endl;
            }
          }
        }
        set_zero(i_errors);
        state = DETECT_NORMAL_THUMB;
      }

      }
    }
    else if (state == DETECT_NORMAL_THUMB)
    {
      temp_finger_command_torques[0] = detect_surface_normal(robot, link_names[0], poses[0].translation(), current_finger_position[0], detect_states[0], detect_velocity_record[0], contact_points[0], normals[0], static_counter[0], i_errors[0], 65.0);
      for ( int i = 1; i < NUM_OF_FINGERS_USED; i++)
      {
        temp_finger_command_torques[i] = compute_position_cmd_torques(robot, link_names[i], poses[i].translation(),current_finger_position[i], 65.0, i_errors[i]);
      }
      for (int j = NUM_OF_FINGERS_USED; j < NUM_OF_FINGERS_IN_MODEL; j++)
      {
        temp_finger_command_torques[j] = compute_position_cmd_torques(robot, link_names[j], poses[j].translation(), Vector3d(0.1, -0.065, -0.07), 65.0);
        // finger_command_torques[j].block(6 + 4 * j, 0, 4, 1) = temp_finger_command_torques[j].block(6 + 4 * j,0,4,1);
      }
      finger_command_torques = block_torque(temp_finger_command_torques);
      

      if (normals[0].norm() > 0.5)
      {
        set_zero(i_errors);
        cout << "thumb normals detected" << endl;
        state = OPTIMIZE;
        for ( int j = 0; j < contact_points[0].size() ; j++)
          {
            for (int k = 0; k < 3; k++)
            {
              output << contact_points[0][j][k]<<endl;
            }
          }
        output.close();
        if(predefined_normal_flag == true)
        {
          normals[0] = Vector3d(1.0, 0.0, 0.0);
        }
      }
    }
    else if (state == OPTIMIZE)  // this optimization code only optimizes the position of the thumb
    {
      // the vector to record the score in optimization algorithm for the 5 trial points of the thumb
      vector<double> scores;
      // the forces for the 3 fingers corresponding to the 5 thum positions
      vector< vector <Vector3d> > forces_candidates;
      vector<Vector3d> temp_vector;
      // -----------initialization-------------------------------
      for (int i = 0; i < NUM_OF_FINGERS_USED; i++)
      {
        temp_vector.push_back(Vector3d::Zero());
      }
      for( int i = 0; i < 5; i++)
      {
        for( int j = 0; j < NUM_OF_FINGERS_USED; j++)
        {
          forces_candidates.push_back(temp_vector);
        }
      }
      double temp_score = 0;
      // ------------------------------------------------------
      // redis communication
      for(int i = 0; i < 5; ) // 5 means the num of points probbed
      {  
        redis_cli.getCommandIs(PYTHON_START_FLAG_KEY);
        bool python_start_flag = stoi(redis_cli.reply_->str);
        if (python_start_flag == false)  // which means the python code is ready to receive any message
        {
          vector<Vector3d> contact_positions_sent;
          contact_positions_sent.push_back(contact_points[0][i]); // the thumb position
          for (int j = 1; j < NUM_OF_FINGERS_USED; j++)  // other-finger positions 
          {
            contact_positions_sent.push_back(contact_points[j][0]);
          }
          send_to_redis(redis_cli, contact_positions_sent, normals);
          python_start_flag = true;
          redis_cli.setCommandIs(PYTHON_START_FLAG_KEY, to_string(python_start_flag));
          while(true)
          {
            usleep(10000);
            redis_cli.getCommandIs(PYTHON_START_FLAG_KEY);
            python_start_flag = stoi(redis_cli.reply_->str);
            if(python_start_flag == false)
            {
              break;
            }

          }
          redis_cli.getCommandIs(SCORE_KEY);
          temp_score = stod(redis_cli.reply_->str);
          cout << temp_score << endl;
          scores.push_back(temp_score);

          redis_cli.getEigenMatrixDerived(FORCE_1_KEY, forces_candidates[i][0]);
          redis_cli.getEigenMatrixDerived(FORCE_2_KEY, forces_candidates[i][1]);
          redis_cli.getEigenMatrixDerived(FORCE_3_KEY, forces_candidates[i][2]);

          i++;
        }
      }
      // -------------------------------------------------------------
      // choose the result
      double minimal_score = 9999;
      int minimal_index = -1;
      for( int i = 0; i < 5; i++)
      {
        if (scores[i] <= minimal_score)
        { minimal_score = scores[i];
          minimal_index = i;
        }
      }
      if (minimal_index == 0 && minimal_score < max_score_threshold && minimal_score > min_score_threshold) // has already reached the local minimum
      {
        optimal_positions[0] = contact_points[0][minimal_index];
        optimal_forces = forces_candidates[minimal_index];
        CoM_of_object = (2 * current_finger_position[0] + current_finger_position[1] + current_finger_position[2]) / 4;  
        state = CHECK;
        cout << "the forces" << endl;
        for(int i = 0; i < 3; i++)
        {
          cout << optimal_forces[i] << endl << endl;
        }
      } 
      else if ((scores[0] - minimal_score < 5 * mass) && scores[0] < max_score_threshold && scores[0] > min_score_threshold)
      {
        cout << "hasn't reach the optimal state but already close enough!" << endl;
        optimal_positions[0] = contact_points[0][0];
        optimal_forces = forces_candidates[0];
        CoM_of_object = (2 * current_finger_position[0] + current_finger_position[1] + current_finger_position[2]) / 4;  
        state = CHECK;
        cout << "the forces" << endl;
        for(int i = 0; i < 3; i++)
        {
          cout << optimal_forces[i] << endl << endl;
        }
      }
      else if (reprob_times >= 5)
      {
        cout << "has reprobed too many times, just randomly give it a shot" << endl;
        optimal_positions[0] = contact_points[0][0];
        optimal_forces = forces_candidates[0];        
        CoM_of_object = (2 * current_finger_position[0] + current_finger_position[1] + current_finger_position[2]) / 4;
        state = CHECK;
        cout << "the forces" << endl;
        for(int i = 0; i < 3; i++)
        {
          cout << optimal_forces[i] << endl << endl;
        }
      }
      else
      { 
        current_centroid = contact_points[0][minimal_index];
        detect_states[0] = 0;
        detect_velocity_record[0][0] = 0;
        detect_velocity_record[0][1] = 0;
        normals[0] = Vector3d::Zero();
        contact_points[0].clear();
        contact_points[0].push_back(current_centroid);
        static_counter[0];
        state = RE_PROB;
        cout << "you are in the re_prob state" << endl;
      } 
    
      
    }

    else if (state == RE_PROB)  // essentially the same as thumb prob state
    {
      temp_finger_command_torques[0] = detect_surface_normal(robot, link_names[0], poses[0].translation(), current_centroid, detect_states[0], detect_velocity_record[0], contact_points[0], normals[0], static_counter[0], i_errors[0], 65.0);

      // Keep the position of other fingers
      for (int i = 1; i < NUM_OF_FINGERS_USED; i++)
      {
        temp_finger_command_torques[i] = compute_position_cmd_torques(robot, link_names[i], poses[i].translation(), contact_points[i][0], 65.0);
        // finger_command_torques[j].block(6 + 4 * j, 0, 4, 1) = temp_finger_command_torques[j].block(6 + 4 * j,0,4,1);sai2
      }
      for ( int i = NUM_OF_FINGERS_USED; i < NUM_OF_FINGERS_IN_MODEL; i++)
      {
        temp_finger_command_torques[i] = compute_position_cmd_torques(robot, link_names[i], poses[i].translation(), Vector3d(0.1, -0.075, -0.06), 65.0);
      }

      finger_command_torques = block_torque(temp_finger_command_torques);
      if(normals[0].norm() > 0.8)
      { 
        cout << "do the optimization again" << endl;
        reprob_times++;
        set_zero(i_errors);
        if(predefined_normal_flag == true)
        {
          normals[0] = Vector3d(1.0, 0.0, 0.0);
          normals[1] = Vector3d(-1.0, 0.0, 0.0);
          normals[2] = Vector3d(-1.0, 0.0, 0.0);  
        }
        
        state = OPTIMIZE;
      }
    }


    else if (state == CHECK) // it's actually an empty state, we can implement a checking to it
    {
      cout << "you already found the best point!!!" << endl;

      // check whether we can achieve 2 finger contact.
      if (check_2_finger_grasp(current_finger_position, normals, FRICTION_COEFFICIENT))
      {
        cout << "positioning the fingers to be prepared for the grasp!" << endl;     
        state = POSITION_FINGERS;
      }
      else
      {
        cout << "positioning the fingers to be prepared for the grasp!" << endl;     
        state = POSITION_FINGERS;
      }


    }

    else if (state == POSITION_FINGERS) // position the fingers to the desired position and get ready for a grasp
    { 
      for(int j = 0; j < NUM_OF_FINGERS_USED; j++)
      {
        temp_finger_command_torques[j] = compute_position_cmd_torques(robot, link_names[j], poses[j].translation(), optimal_positions[j], 65.0, i_errors[j], 0.01,50/50 );
      }
      
      for (int j = NUM_OF_FINGERS_USED; j < NUM_OF_FINGERS_IN_MODEL; j++)
      {
        temp_finger_command_torques[j] = compute_position_cmd_torques(robot, link_names[j], poses[j].translation(), Vector3d(0.1, -0.075, -0.06), 65.0, i_errors[j], 0.01, 50/50);
        finger_command_torques[j].block(6 + 4 * j, 0, 4, 1) = temp_finger_command_torques[j].block(6 + 4 * j, 0,4,1);
      }
      
      finger_command_torques = block_torque(temp_finger_command_torques);

      double sum_of_errors = 0.0;
      for (int i = 0; i < NUM_OF_FINGERS_USED; i ++)
      {
        robot->position(current_finger_position[i], link_names[i], poses[i].translation());
        sum_of_errors += (current_finger_position[i] - optimal_positions[i]).norm();
      }

      if (sum_of_errors < 0.007)
      {
        cout << "Here are the errors: " << sum_of_errors << endl;
        cout << "The current contact_points are:" << endl;
        cout << current_finger_position[0] << endl;
        cout << current_finger_position[1] << endl;
        cout << current_finger_position[2] << endl;

        cout << "The optimal_positions are: "<< endl;
        cout << optimal_positions[0] << endl;
        cout << optimal_positions[1] << endl;
        cout << optimal_positions[2] << endl;

        //  since the actual position may not be the same as the desired position, we may want to do the optimization again to get the 
        redis_cli.getCommandIs(PYTHON_START_FLAG_KEY);
        bool python_start_flag = stoi(redis_cli.reply_->str);
        if (python_start_flag == false)  // which means the python code is ready to receive any message
        {
          vector<Vector3d> orig_opt_forces = {Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero()};
          // cout << "!!!!!!!!!!!!!!!"<< i <<endl;
          vector<Vector3d> contact_positions_sent;
          for (int j = 0; j < NUM_OF_FINGERS_USED; j++)  // other-finger positions 
          {
            contact_positions_sent.push_back(current_finger_position[j]);
          }
          send_to_redis(redis_cli, contact_positions_sent, normals);
          python_start_flag = true;
          redis_cli.setCommandIs(PYTHON_START_FLAG_KEY, to_string(python_start_flag));
          redis_cli.getEigenMatrixDerived(FORCE_1_KEY, optimal_forces[0]);  // update the applting forces to be based on the real contact positions
          redis_cli.getEigenMatrixDerived(FORCE_2_KEY, optimal_forces[1]);
          redis_cli.getEigenMatrixDerived(FORCE_3_KEY, optimal_forces[2]);
          cout << "the current position optimal forces are: "<< endl;
          cout << optimal_forces[0] << endl;
          cout << optimal_forces[1] << endl;
          cout << optimal_forces[2] << endl;
        }
        cout << "start applying force" << endl;

        set_zero(i_errors);
        state = APPLY_FORCE;
      }
    }

    else if (state == APPLY_FORCE)
    {
      // optimal_forces[0][2] = 0;
      // optimal_forces[1][2] = 0;
      // optimal_forces[2][2] = 0;
      for( int i = 0; i < NUM_OF_FINGERS_USED; i++)
      {
        temp_finger_command_torques[i] = compute_force_cmd_torques(robot, link_names[i], poses[i].translation(), optimal_forces[i]);
        bounce_back_torques[i] = bounce_back(robot, link_names[i], poses[i].translation(), optimal_positions[i], optimal_forces[i], 65.0, 0.5);
        temp_finger_command_torques[i] += bounce_back_torques[i];
      }
      /*      for(int j = 1; j < NUM_OF_FINGERS_USED; j++)
      {
        temp_finger_command_torques[j] = compute_position_cmd_torques(robot, link_names[j], poses[j].translation(), optimal_positions[j], 130.0, i_errors[j], 0.001, 2.5);
      }
      robot->position(current_finger_position[0], link_names[0], poses[0].translation());
      // ptimal_forces[0].dot(normals[0]) * normals[0]
      temp_finger_command_torques[0] = compute_force_cmd_torques(robot, link_names[0], poses[0].translation(), 1.5 * optimal_forces[0]);
      bounce_back_torques[0] = bounce_back(robot, link_names[0], poses[0].translation(), optimal_positions[0], optimal_forces[0], 65.0, 0.5);
      temp_finger_command_torques[0] += bounce_back_torques[0];*/
      // for(int j = 0; j < NUM_OF_FINGERS_USED; j++)
      // {
      //   finger_command_torques[j].block(6 + 4 * j ,0 ,4, 1) = temp_finger_command_torques[j].block(6 + 4 * j, 0 ,4 ,1 );
      // }

      for (int j = NUM_OF_FINGERS_USED; j < NUM_OF_FINGERS_IN_MODEL; j++)
      {
        temp_finger_command_torques[j] = compute_position_cmd_torques(robot, link_names[j], poses[j].translation(), Vector3d(0.1, -0.075, -0.06), 10.0);
      }
      finger_command_torques = block_torque(temp_finger_command_torques);
    }
    else if (state == TEST)
    {
      VectorXd desired_joint_angles = VectorXd::Zero(dof);
      desired_joint_angles = driver_to_sai2(origin_config);
      // desired_joint_angles[6] = - 1.57;
      joint_torques = compute_joint_cmd_torques(robot, desired_joint_angles);
      // redis_cli.setEigenMatrixDerived(JOINT_ANGLE_KEY, robot->_q);
    }

    loop_counter++;
    // command_torques = joint_torques;
  command_torques = finger_command_torques[0] + finger_command_torques[1] \
  + finger_command_torques[2] + finger_command_torques[3] + joint_torques;
  if (friction_compensation_flag == true)
  {
    for (int i = 0; i < dof; i++)
    {
      if (command_torques[i] > 0)
      {
        command_torques[i] += friction_compensation_pos[i];
      }
      else if(command_torques[i] < 0)
      {
        command_torques[i] += - friction_compensation_pos[i];
      }
    }

    // compensate the stiffness in the thumb
    if(command_torques[8] > 0)
    {
      command_torques[8] += 0.05;
    }
  }
  command_torques += gravity;

  // command_torques = gravity;
    // command_torques << 0,0,0,0,0,0,
    // 0.0,0.0,0.0,0.0,
    // 0.0,0.0,0.0,0.0,
    // 0.0,0.0,0.0,0.0,
    // 0.0,0.0,0.0,0.0;
  // cout << "here's the command torque:" << command_torques <<endl << endl;
  sai2_to_driver(command_torques,control_torque);
    // reset to 0
  for(int i =0; i < NUM_OF_FINGERS_IN_MODEL; i++)
  {
    temp_finger_command_torques[i].setZero();
    finger_command_torques[i].setZero();
  }
  command_torques.setZero();
  joint_torques.setZero();
  }

  command_torques.setZero();
  sai2_to_driver(command_torques,control_torque);
}

/////////////////////////////////////////////////////////////////////////////////////////
// CAN communication thread
static void* ioThreadProc(void* inst)
{
  char id_des;
  char id_cmd;
  char id_src;
  int len;
  unsigned char data[8];
  unsigned char data_return = 0;
  int i;

  while (ioThreadRun)
    {
      /* wait for the event */
      while (0 == get_message(CAN_Ch, &id_cmd, &id_src, &id_des, &len, data, FALSE))
	{
	  switch (id_cmd)
	    {
	    case ID_CMD_QUERY_ID:
	      {
		printf(">CAN(%d): AllegroHand revision info: 0x%02x%02x\n", CAN_Ch, data[3], data[2]);
		printf("                      firmware info: 0x%02x%02x\n", data[5], data[4]);
		printf("                      hardware type: 0x%02x\n", data[7]);
	      }
	      break;

	    case ID_CMD_AHRS_POSE:
	      {
		printf(">CAN(%d): AHRS Roll : 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
		printf("               Pitch: 0x%02x%02x\n", data[2], data[3]);
		printf("               Yaw  : 0x%02x%02x\n", data[4], data[5]);
	      }
	      break;

	    case ID_CMD_AHRS_ACC:
	      {
		printf(">CAN(%d): AHRS Acc(x): 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
		printf("               Acc(y): 0x%02x%02x\n", data[2], data[3]);
		printf("               Acc(z): 0x%02x%02x\n", data[4], data[5]);
	      }
	      break;

	    case ID_CMD_AHRS_GYRO:
	      {
		printf(">CAN(%d): AHRS Angular Vel(x): 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
		printf("               Angular Vel(y): 0x%02x%02x\n", data[2], data[3]);
		printf("               Angular Vel(z): 0x%02x%02x\n", data[4], data[5]);
	      }
	      break;

	    case ID_CMD_AHRS_MAG:
	      {
		printf(">CAN(%d): AHRS Magnetic Field(x): 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
		printf("               Magnetic Field(y): 0x%02x%02x\n", data[2], data[3]);
		printf("               Magnetic Field(z): 0x%02x%02x\n", data[4], data[5]);
	      }
	      break;

	    case ID_CMD_QUERY_CONTROL_DATA:
	      {
		if (id_src >= ID_DEVICE_SUB_01 && id_src <= ID_DEVICE_SUB_04)
		  {
		    vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 0] = (int)(data[0] | (data[1] << 8));
		    vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 1] = (int)(data[2] | (data[3] << 8));
		    vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 2] = (int)(data[4] | (data[5] << 8));
		    vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 3] = (int)(data[6] | (data[7] << 8));
		    data_return |= (0x01 << (id_src-ID_DEVICE_SUB_01));
		    recvNum++;
		  }
		if (data_return == (0x01 | 0x02 | 0x04 | 0x08))
		  {
		    // convert encoder count to joint angle
        for (int i = 0; i < MAX_DOF; i++)    
          {
            dq[i] = (q[i] - q_prev[i]) / delT;
            dq_filter_input[i] = dq[i] / gain;
            dq_filtered[i] = dq_prev_prev_filter_input[i] + 2*dq_prev_filter_input[i] + dq_filter_input[i] + filter_coeffs[0]*dq_prev_prev_filtered[i] + filter_coeffs[1]*dq_prev_filtered[i];
          }
        for (int i = 0; i < MAX_DOF; i++)
        {
            q_prev[i] = q[i];
            dq_prev_prev_filter_input[i] = dq_prev_filter_input[i];
            dq_prev_filter_input[i] = dq_filter_input[i];
            dq_prev_prev_filtered[i] = dq_prev_filtered[i];
            dq_prev_filtered[i] = dq_filtered[i];
        }
		    for (i=0; i<MAX_DOF; i++)
		      {
			q[i] = (double)(vars.enc_actual[i]*enc_dir[i]-32768-enc_offset[i])*(333.3/65536.0)*(3.141592/180.0) + q_offset[i];
		      }

        // compute joint torque
          ComputeGravityTorque();
          for (int j = 0; j < MAX_DOF; j++) 
          {
            tau_des[j] = control_torque[j] + gravity_tourque[j];
          }

  /*        cout << "here is the tau_des: \n" ;
        for (int j=0; j < MAX_DOF; j++)
        {
          cout << tau_des[j] << endl;
        }
        cout << endl;*/

		    // convert desired torque to desired current and PWM count

	    for (i=0; i<MAX_DOF; i++)
	      {
			   cur_des[i] = tau_des[i] * motor_dir[i];
			   if (cur_des[i] > 1.0) cur_des[i] = 1.0;
			   else if (cur_des[i] < -1.0) cur_des[i] = -1.0;
	      }

		    // send torques
	    for (int i=0; i<4;i++)
	      {
			// the index order for motors is different from that of encoders
			switch (HAND_VERSION)
			  {
			  case 1:
			  case 2:
			    vars.pwm_demand[i*4+3] = (short)(cur_des[i*4+0]*tau_cov_const_v2);
			    vars.pwm_demand[i*4+2] = (short)(cur_des[i*4+1]*tau_cov_const_v2);
			    vars.pwm_demand[i*4+1] = (short)(cur_des[i*4+2]*tau_cov_const_v2);
			    vars.pwm_demand[i*4+0] = (short)(cur_des[i*4+3]*tau_cov_const_v2);
			    break;

			  case 3:
			  default:
			    vars.pwm_demand[i*4+3] = (short)(cur_des[i*4+0]*tau_cov_const_v3);
			    vars.pwm_demand[i*4+2] = (short)(cur_des[i*4+1]*tau_cov_const_v3);
			    vars.pwm_demand[i*4+1] = (short)(cur_des[i*4+2]*tau_cov_const_v3);
			    vars.pwm_demand[i*4+0] = (short)(cur_des[i*4+3]*tau_cov_const_v3);
			    break;
			  }
			write_current(CAN_Ch, i, &vars.pwm_demand[4*i]);
			usleep(5);
		      }
		    sendNum++;
		    curTime += delT;

		    data_return = 0;
		  }
	      }
	      break;
	    }
	}
    }

  return NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Application main-loop. It handles the commands from rPanelManipulator and keyboard events
void MainLoop()
{

  bool bRun = true;

  if(!initializeRedis()) {cout<<"Redis initialization failed";exit(1);}
  int prev_c = 0;
  pBHand->SetMotionType(eMotionType_GRAVITY_COMP);
  while (bRun)
    {
      int c = Getch();    
      // int c = getCommandFromRedis();
      if (prev_c != c)
      { 
      custom_PD = false;
		  printf("%c\n",c);
		  prev_c = c;
	      switch (c)
	        {
	        case 'q':
  		  if (pBHand) pBHand->SetMotionType(eMotionType_NONE);
  		  bRun = false;
  		  break;
      case 'Q':
        if (pBHand) pBHand->SetMotionType(eMotionType_NONE);
        bRun = false;
        break;
          case 'o':
      if (pBHand) pBHand->SetMotionType(eMotionType_NONE);
      break;

	        }
      // usleep(100000);
	    }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Compute control torque for each joint using BHand library
void ComputeGravityTorque()
{
  if (!pBHand) return;
  pBHand->SetJointPosition(q); // tell BHand library the current joint positions
//  pBHand->SetJointDesiredPosition(q_des);  // this line isn't needed if we are using some grasping mode defined by the library
  pBHand->UpdateControl(0);
  pBHand->GetJointTorque(gravity_tourque);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Compute control torque for each joint using custom pd controller

/////////////////////////////////////////////////////////////////////////////////////////
// Open a CAN data channel
bool OpenCAN()
{
#if defined(PEAKCAN)
  CAN_Ch = GetCANChannelIndex(_T("USBBUS1"));
#elif defined(IXXATCAN)
  CAN_Ch = 1;
#elif defined(SOFTINGCAN)
  CAN_Ch = 1;
#else
  CAN_Ch = 1;
#endif
  CAN_Ch = GetCANChannelIndex(_T("USBBUS1"));
  printf(">CAN(%d): open\n", CAN_Ch);

  int ret = command_can_open(CAN_Ch);
  if(ret < 0)
    {
      printf("ERROR command_canopen !!! \n");
      return false;
    }

  ioThreadRun = true;

  /* initialize condition variable */
  int ioThread_error = pthread_create(&hThread, NULL, ioThreadProc, 0);
  int sai2_error = pthread_create(&cThread, NULL, sai2, 0);
  if (ioThread_error)
    {
      printf("error, the io thread starting procedure failed.\n");
    }
    
  printf(">CAN: starts listening CAN frames\n");

  printf(">CAN: query system id\n");
  ret = command_can_query_id(CAN_Ch);
  if(ret < 0)
    {
      printf("ERROR command_can_query_id !!! \n");
      command_can_close(CAN_Ch);
      return false;
    }

  printf(">CAN: AHRS set\n");
  ret = command_can_AHRS_set(CAN_Ch, AHRS_RATE_100Hz, AHRS_MASK_POSE | AHRS_MASK_ACC);
  if(ret < 0)
    {
      printf("ERROR command_can_AHRS_set !!! \n");
      command_can_close(CAN_Ch);
      return false;
    }

  printf(">CAN: system init\n");
  ret = command_can_sys_init(CAN_Ch, 3/*msec*/);
  if(ret < 0)
    {
      printf("ERROR command_can_sys_init !!! \n");
      command_can_close(CAN_Ch);
      return false;
    }

  printf(">CAN: start periodic communication\n");
  ret = command_can_start(CAN_Ch);

  if(ret < 0)
    {
      printf("ERROR command_can_start !!! \n");
      command_can_stop(CAN_Ch);
      command_can_close(CAN_Ch);
      return false;
    }

  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Close CAN data channel
void CloseCAN()
{
  printf(">CAN: stop periodic communication\n");
  int ret = command_can_stop(CAN_Ch);
  if(ret < 0)
    {
      printf("ERROR command_can_stop !!! \n");
    }

  if (ioThreadRun)
    {
      printf(">CAN: stoped listening CAN frames\n");
      ioThreadRun = false;
      int status;
      pthread_join(hThread, (void **)&status);
      hThread = 0;
    }

  printf(">CAN(%d): close\n", CAN_Ch);
  ret = command_can_close(CAN_Ch);
  if(ret < 0) printf("ERROR command_can_close !!! \n");
}

/////////////////////////////////////////////////////////////////////////////////////////
// Load and create grasping algorithm
bool CreateBHandAlgorithm()
{
  if (RIGHT_HAND)
    pBHand = bhCreateRightHand();
  else
    pBHand = bhCreateLeftHand();

  if (!pBHand) return false;
  pBHand->SetTimeInterval(delT);
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Destroy grasping algorithm
void DestroyBHandAlgorithm()
{
  if (pBHand)
    {
  #ifndef _DEBUG
        delete pBHand;
  #endif
        pBHand = NULL;
    }
}

////////////////////////////////////////////////////////////////////////////////////////
// Print program information and keyboard instructions
void PrintInstruction()
{
  printf("--------------------------------------------------\n");
  printf("myAllegroHand: ");
  if (RIGHT_HAND) printf("Right Hand, v%i.x\n\n", HAND_VERSION); else printf("Left Hand, v%i.x\n\n", HAND_VERSION);

  printf("Keyboard Commands:\n");
  printf("H: Home Position (PD control)\n");
  printf("R: Ready Position (used before grasping)\n");
  printf("G: Three-Finger Grasp\n");
  printf("K: Four-Finger Grasp\n");
  printf("P: Two-finger pinch (index-thumb)\n");
  printf("M: Two-finger pinch (middle-thumb)\n");
  printf("E: Envelop Grasp (all fingers)\n");
  printf("A: Gravity Compensation\n\n");

  printf("O: Servos OFF (any grasp cmd turns them back on)\n");
  printf("Q: Quit this program\n");

  printf("--------------------------------------------------\n\n");
}
/////////////////////////////////////////////////////////////////////////////////////////
// Get channel index for Peak CAN interface
int GetCANChannelIndex(const TCHAR* cname)
{
  if (!cname) return 0;

  if (!_tcsicmp(cname, _T("0")) || !_tcsicmp(cname, _T("PCAN_NONEBUS")) || !_tcsicmp(cname, _T("NONEBUS")))
    return 0;
  else if (!_tcsicmp(cname, _T("1")) || !_tcsicmp(cname, _T("PCAN_ISABUS1")) || !_tcsicmp(cname, _T("ISABUS1")))
    return 1;
  else if (!_tcsicmp(cname, _T("2")) || !_tcsicmp(cname, _T("PCAN_ISABUS2")) || !_tcsicmp(cname, _T("ISABUS2")))
    return 2;
  else if (!_tcsicmp(cname, _T("3")) || !_tcsicmp(cname, _T("PCAN_ISABUS3")) || !_tcsicmp(cname, _T("ISABUS3")))
    return 3;
  else if (!_tcsicmp(cname, _T("4")) || !_tcsicmp(cname, _T("PCAN_ISABUS4")) || !_tcsicmp(cname, _T("ISABUS4")))
    return 4;
  else if (!_tcsicmp(cname, _T("5")) || !_tcsicmp(cname, _T("PCAN_ISABUS5")) || !_tcsicmp(cname, _T("ISABUS5")))
    return 5;
  else if (!_tcsicmp(cname, _T("7")) || !_tcsicmp(cname, _T("PCAN_ISABUS6")) || !_tcsicmp(cname, _T("ISABUS6")))
    return 6;
  else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS7")) || !_tcsicmp(cname, _T("ISABUS7")))
    return 7;
  else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS8")) || !_tcsicmp(cname, _T("ISABUS8")))
    return 8;
  else if (!_tcsicmp(cname, _T("9")) || !_tcsicmp(cname, _T("PCAN_DNGBUS1")) || !_tcsicmp(cname, _T("DNGBUS1")))
    return 9;
  else if (!_tcsicmp(cname, _T("10")) || !_tcsicmp(cname, _T("PCAN_PCIBUS1")) || !_tcsicmp(cname, _T("PCIBUS1")))
    return 10;
  else if (!_tcsicmp(cname, _T("11")) || !_tcsicmp(cname, _T("PCAN_PCIBUS2")) || !_tcsicmp(cname, _T("PCIBUS2")))
    return 11;
  else if (!_tcsicmp(cname, _T("12")) || !_tcsicmp(cname, _T("PCAN_PCIBUS3")) || !_tcsicmp(cname, _T("PCIBUS3")))
    return 12;
  else if (!_tcsicmp(cname, _T("13")) || !_tcsicmp(cname, _T("PCAN_PCIBUS4")) || !_tcsicmp(cname, _T("PCIBUS4")))
    return 13;
  else if (!_tcsicmp(cname, _T("14")) || !_tcsicmp(cname, _T("PCAN_PCIBUS5")) || !_tcsicmp(cname, _T("PCIBUS5")))
    return 14;
  else if (!_tcsicmp(cname, _T("15")) || !_tcsicmp(cname, _T("PCAN_PCIBUS6")) || !_tcsicmp(cname, _T("PCIBUS6")))
    return 15;
  else if (!_tcsicmp(cname, _T("16")) || !_tcsicmp(cname, _T("PCAN_PCIBUS7")) || !_tcsicmp(cname, _T("PCIBUS7")))
    return 16;
  else if (!_tcsicmp(cname, _T("17")) || !_tcsicmp(cname, _T("PCAN_PCIBUS8")) || !_tcsicmp(cname, _T("PCIBUS8")))
    return 17;
  else if (!_tcsicmp(cname, _T("18")) || !_tcsicmp(cname, _T("PCAN_USBBUS1")) || !_tcsicmp(cname, _T("USBBUS1")))
    return 18;
  else if (!_tcsicmp(cname, _T("19")) || !_tcsicmp(cname, _T("PCAN_USBBUS2")) || !_tcsicmp(cname, _T("USBBUS2")))
    return 19;
  else if (!_tcsicmp(cname, _T("20")) || !_tcsicmp(cname, _T("PCAN_USBBUS3")) || !_tcsicmp(cname, _T("USBBUS3")))
    return 20;
  else if (!_tcsicmp(cname, _T("21")) || !_tcsicmp(cname, _T("PCAN_USBBUS4")) || !_tcsicmp(cname, _T("USBBUS4")))
    return 21;
  else if (!_tcsicmp(cname, _T("22")) || !_tcsicmp(cname, _T("PCAN_USBBUS5")) || !_tcsicmp(cname, _T("USBBUS5")))
    return 22;
  else if (!_tcsicmp(cname, _T("23")) || !_tcsicmp(cname, _T("PCAN_USBBUS6")) || !_tcsicmp(cname, _T("USBBUS6")))
    return 23;
  else if (!_tcsicmp(cname, _T("24")) || !_tcsicmp(cname, _T("PCAN_USBBUS7")) || !_tcsicmp(cname, _T("USBBUS7")))
    return 24;
  else if (!_tcsicmp(cname, _T("25")) || !_tcsicmp(cname, _T("PCAN_USBBUS8")) || !_tcsicmp(cname, _T("USBBUS8")))
    return 25;
  else if (!_tcsicmp(cname, _T("26")) || !_tcsicmp(cname, _T("PCAN_PCCBUS1")) || !_tcsicmp(cname, _T("PCCBUS1")))
    return 26;
  else if (!_tcsicmp(cname, _T("27")) || !_tcsicmp(cname, _T("PCAN_PCCBUS2")) || !_tcsicmp(cname, _T("PCCBUS2")))
    return 271;
  else
    return 0;
}


/////////////////////////////////////////////////////////////////////////////////////////
// Program main
int main(int argc, TCHAR* argv[])
{
  PrintInstruction();

  memset(&vars, 0, sizeof(vars));
  memset(q, 0, sizeof(q));
  memset(q_des, 0, sizeof(q_des));
  memset(tau_des, 0, sizeof(tau_des));
  memset(cur_des, 0, sizeof(cur_des));
  curTime = 0.0;

  if (CreateBHandAlgorithm() && OpenCAN())
    {
      MainLoop();
    }

  CloseCAN();
  DestroyBHandAlgorithm();

  return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////
// functions from Sai2 program
VectorXd compute_position_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position, double kp)
{

  double kv = sqrt(kp)/3.0;
  int dof = robot->dof();
  Vector3d current_position; // in robot frame
  Vector3d current_velocity;
  MatrixXd Jv = MatrixXd::Zero(3,dof);
  robot->Jv(Jv, link, pos_in_link);
  robot->position(current_position, link, pos_in_link);
  robot->linearVelocity(current_velocity, link, pos_in_link);
  VectorXd torque = VectorXd::Zero(dof);
  torque = Jv.transpose()*(kp * (desired_position - current_position) - kv * current_velocity);
  if (dynamic_consistence_flag == true)
  {
    MatrixXd N_prec = MatrixXd::Identity(dof,dof);
    MatrixXd N = MatrixXd::Zero(dof, dof);
    robot->nullspaceMatrix(N, Jv, N_prec);
    VectorXd origin = driver_to_sai2(origin_config);
    torque += N.transpose() * 0.1 * (origin - robot->_q);
  }
  else if (joint_limit_avoidance_flag == true)
  {
    MatrixXd N_prec = MatrixXd::Identity(dof,dof);
    MatrixXd N = MatrixXd::Zero(dof, dof);
    robot->nullspaceMatrix(N, Jv, N_prec);
    VectorXd satu_avoid_torque = VectorXd::Zero(dof);
    for ( int i = 0; i < dof; i++)
    {
      satu_avoid_torque[i] = avoidance_coefficient * (- robot->_q[i] + 0.785);
    }
    satu_avoid_torque[6] = 0;
    satu_avoid_torque[7] = 0;
    satu_avoid_torque[10] = 0;
    satu_avoid_torque[14] = 0;
    satu_avoid_torque[18] = 0;
    torque += N.transpose() * satu_avoid_torque;

  }
  return torque;
}
VectorXd compute_position_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position, double kp, Vector3d& i_error)
{
  double kv = sqrt(kp)/3.0;
  double ki = 0.0015;
  int dof = robot->dof();
  
  if(i_error.norm() > 1000)
    i_error = Vector3d::Zero();

  Vector3d current_position; // in robot frame
  Vector3d current_velocity;
  MatrixXd Jv = MatrixXd::Zero(3,dof);
  robot->Jv(Jv, link, pos_in_link);
  robot->position(current_position, link, pos_in_link);
  Vector3d error = desired_position - current_position;
  i_error += error;
  robot->linearVelocity(current_velocity, link, pos_in_link);
  VectorXd torque = VectorXd::Zero(dof);
  torque = Jv.transpose()*(kp * (desired_position - current_position) - kv * current_velocity + ki * i_error);
  if (dynamic_consistence_flag == true)
  {
    MatrixXd N_prec = MatrixXd::Identity(dof,dof);
    MatrixXd N = MatrixXd::Zero(dof, dof);
    robot->nullspaceMatrix(N, Jv, N_prec);
    VectorXd origin = driver_to_sai2(origin_config);
    torque += N.transpose() * 0.1 * (origin - robot->_q);
  }
  else if (joint_limit_avoidance_flag == true)
  {
    MatrixXd N_prec = MatrixXd::Identity(dof,dof);
    MatrixXd N = MatrixXd::Zero(dof, dof);
    robot->nullspaceMatrix(N, Jv, N_prec);
    VectorXd satu_avoid_torque = VectorXd::Zero(dof);
    for ( int i = 0; i < dof; i++)
    {
      satu_avoid_torque[i] = avoidance_coefficient * (- robot->_q[i] + 0.785);
    }
    satu_avoid_torque[6] = 0;
    satu_avoid_torque[7] = 0;
    satu_avoid_torque[10] = 0;
    satu_avoid_torque[14] = 0;
    satu_avoid_torque[18] = 0;
    torque += N.transpose() * satu_avoid_torque;

  }
  return torque;
}
VectorXd compute_position_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position, double kp, Vector3d& i_error, double ki, double kv)
{
  if (link == "finger0-link3")
  {
    desired_position[2] = min(desired_position[2], -0.105);
  }
  int dof = robot->dof();
  if(i_error.norm() > 1000)
  {
    i_error = Vector3d::Zero();
  }
  Vector3d current_position; // in robot frame
  Vector3d current_velocity;
  MatrixXd Jv = MatrixXd::Zero(3,dof);
  robot->Jv(Jv, link, pos_in_link);
  robot->position(current_position, link, pos_in_link);
  Vector3d error = desired_position - current_position;
  i_error += error;
  robot->linearVelocity(current_velocity, link, pos_in_link);
  VectorXd torque = VectorXd::Zero(dof);
  torque = Jv.transpose()*(kp*(desired_position - current_position) - kv * current_velocity + ki * i_error);
  if (dynamic_consistence_flag == true)
  {
    MatrixXd N_prec = MatrixXd::Identity(dof,dof);
    MatrixXd N = MatrixXd::Zero(dof, dof);
    robot->nullspaceMatrix(N, Jv, N_prec);
    VectorXd origin = driver_to_sai2(origin_config);
    torque += N.transpose() * 0.1 * (origin - robot->_q);
  }
  else if (joint_limit_avoidance_flag == true)
  {
    MatrixXd N_prec = MatrixXd::Identity(dof,dof);
    MatrixXd N = MatrixXd::Zero(dof, dof);
    robot->nullspaceMatrix(N, Jv, N_prec);
    VectorXd satu_avoid_torque = VectorXd::Zero(dof);
    for ( int i = 0; i < dof; i++)
    {
      satu_avoid_torque[i] = avoidance_coefficient * (- robot->_q[i] + 0.785);
    }
    satu_avoid_torque[6] = 0;
    satu_avoid_torque[7] = 0;
    satu_avoid_torque[10] = 0;
    satu_avoid_torque[14] = 0;
    satu_avoid_torque[18] = 0;
    torque += N.transpose() * satu_avoid_torque;

  }
  return torque;
}
VectorXd compute_force_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position, double force_requeired)
{
  int dof = robot->dof();
  // double force_requeired = 0.001;
  Vector3d current_position; // in robot frame
  Vector3d current_velocity;
  Vector3d desired_force;
  MatrixXd Jv = MatrixXd::Zero(3,dof);
  robot->Jv(Jv, link, pos_in_link);
  robot->position(current_position, link, pos_in_link);
  desired_force = desired_position - current_position;
  desired_force = desired_force / desired_force.norm(); // normalization
  desired_force = desired_force * force_requeired;
  VectorXd torque = VectorXd::Zero(dof);
  torque = Jv.transpose()*desired_force;
  return torque;
}
VectorXd compute_force_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_force)
{
  int dof = robot->dof();
  // double force_requeired = 0.001;
  MatrixXd Jv = MatrixXd::Zero(3,dof);
  robot->Jv(Jv, link, pos_in_link);
  VectorXd torque = VectorXd::Zero(dof);
  torque = Jv.transpose()*desired_force;
  return torque;
}

VectorXd detect_surface_normal(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d original_pos, int& state, deque<double>& velocity_record, vector<Vector3d>& contact_points, Vector3d& normal, int& static_counter, Vector3d& i_error, double kp)
{
  double kv = sqrt(kp)/3.0;
  double ki = 0.001;
  double local_displace_dis = displacement_dis;
  double local_surface_force = surface_force;
  double local_prob_distance = prob_distance;
  double torque_threshold = 0.3;
  VectorXd pregrasp_angles = driver_to_sai2(origin_config);
  int index = 999;
  if (link == "finger0-link3")
  {
    index = 0;
    local_displace_dis = - displacement_dis * 1.5;
    local_surface_force = surface_force * 0.5;
    local_prob_distance = prob_distance * 1.0;
    ki = 0.0033;
  }
  else if(link == "finger1-link3")
  {
    index = 1;
  }
  else if(link == "finger2-link3")
  {
    index = 2;
  }
  else if(link == "finger3-link3")
  {
    index = 3;
  }

  int dof = robot->dof();
  VectorXd torque = VectorXd::Zero(dof);
  Vector3d current_position = Vector3d::Zero();
  robot->position(current_position, link, pos_in_link);

    // Vector3d desired_position = displacement_dis*(original_pos - CoM_of_object) / (original_pos - CoM_of_object).norm() + \
    // original_pos + Vector3d(0.0, 0.0, prob_distance);
  if (state == 0)
  {
    torque = compute_joint_cmd_torques_one_finger(robot, pregrasp_angles, index);
    if (torque.norm() < torque_threshold)
    {
      delay_counter++;
      if (delay_counter > 20000)
      {
        state = 1;
        delay_counter = 0;
      }
    }

  }

  if(state == 1) // just start from the initial centroid position
  {
    Vector3d desired_position = local_displace_dis * Vector3d(1.0,0.0,0.0) + original_pos + Vector3d(0.0, 0.0, local_prob_distance);
    torque = compute_position_cmd_torques(robot, link, pos_in_link, desired_position, kp, i_error, ki, kv);
    if((desired_position - current_position).norm() < 1.5 * POSITION_ERROR_TOLER)
    // if(torque.norm() < 0.07)
    {
      delay_counter++;
      if (delay_counter > 40000)
      {
        cout << link << " finished positioning!!" << endl;
        state = 2;
        delay_counter = 0; 
      }
      
    }
  }
  else if (state == 2) // has reached the first intermediate point
  {
    torque = compute_force_cmd_torques(robot, link, pos_in_link, -2 * local_displace_dis * Vector3d(1.0,0.0,0.0) + original_pos + Vector3d(0.0, 0.0, prob_distance), local_surface_force);
    Vector3d temp_finger_velocity = Vector3d::Zero();
    robot->linearVelocity(temp_finger_velocity, link, pos_in_link);
    velocity_record.pop_front();
    velocity_record.push_back(temp_finger_velocity.norm());
    if(velocity_record[1] < STATIC_THRESHOLD)
    {
      static_counter++;
    }
    if ((velocity_record[1]/velocity_record[0] < CONTACT_COEFFICIENT && velocity_record[0] > MIN_COLLISION_V) || static_counter > 20000)
    {
      state = 3;
      velocity_record.pop_front();
      velocity_record.push_back(0.0);
      velocity_record.pop_front();
      velocity_record.push_back(0.0);
      cout << link <<" contact"<<endl;
      // cout<< "the previous velocity is: " << velocity_record[0] << endl;
      // cout << "the current velocity is: " << velocity_record[1] << endl;
      contact_points.push_back(current_position);
      static_counter = 0;
    }
  }
  else if (state == 3)
  {
    torque = compute_joint_cmd_torques_one_finger(robot, pregrasp_angles, index);
    if (torque.norm() < torque_threshold)
    {
      delay_counter++;
      if (delay_counter > 20000)
      {
        state = 4;
        delay_counter = 0;
      }
    }

  }

  else if(state == 4) 
  {

    // Vector3d desired_position = displacement_dis*(original_pos - CoM_of_object) / (original_pos - CoM_of_object).norm() + \
    // original_pos + Vector3d(0.0, 0.0, -prob_distance);
    Vector3d desired_position = local_displace_dis * Vector3d(1.0,0.0,0.0) + original_pos + Vector3d(0.0, 0.0, -local_prob_distance);

    torque = compute_position_cmd_torques(robot, link, pos_in_link, desired_position, kp, i_error, ki, kv);
    if((desired_position - current_position).norm() < POSITION_ERROR_TOLER)
    // if(torque.norm() < 0.07)
    {
      delay_counter++;
      if (delay_counter > 20000)
      {
        cout << link << " finished positioning!!" << endl;
        state = 5;
        delay_counter = 0; 
      }
    }
  }

  else if (state == 5) // has reached the second intermediate point
  {
    torque = compute_force_cmd_torques(robot, link, pos_in_link, - 2 * local_displace_dis * Vector3d(1.0,0.0,0.0) + original_pos + Vector3d(0.0, 0.0, -prob_distance), local_surface_force);
    Vector3d temp_finger_velocity = Vector3d::Zero();
    robot->linearVelocity(temp_finger_velocity, link, pos_in_link);
    velocity_record.pop_front();
    velocity_record.push_back(temp_finger_velocity.norm());
    if(velocity_record[1] < STATIC_THRESHOLD)
    {
      static_counter++;
    }
    if ((velocity_record[1]/velocity_record[0] < CONTACT_COEFFICIENT && velocity_record[0] > MIN_COLLISION_V) || static_counter > 20000)
    {
      state = 6;
      velocity_record.pop_front();
      velocity_record.push_back(0.0);
      velocity_record.pop_front();
      velocity_record.push_back(0.0);
      cout << link <<" contact"<<endl;
      // cout<< "the previous velocity is: " << velocity_record[0] << endl;
      // cout << "the current velocity is: " << velocity_record[1] << endl;
      contact_points.push_back(current_position); 
      static_counter = 0;
    }
  }
  else if (state == 6)
  {
    torque = compute_joint_cmd_torques_one_finger(robot, pregrasp_angles, index);
    if (torque.norm() < torque_threshold)
    {
      delay_counter++;
      if (delay_counter > 20000)
      {
        state = 7;
        delay_counter = 0;
      }
    }
  }

  else if(state == 7) 
  {
    // Vector3d disp = Vector3d(0.0, 0.0, 0.0);
    // Vector3d origin_disp = (original_pos - CoM_of_object) / (original_pos - CoM_of_object).norm();
    // disp[0] = origin_disp[1]/sqrt(pow(origin_disp[0], 2) + pow(origin_disp[1], 2));
    // disp[1] = - origin_disp[0]/sqrt(pow(origin_disp[0], 2) + pow(origin_disp[1], 2));

    // Vector3d desired_position = displacement_dis*(original_pos - CoM_of_object) / (original_pos - CoM_of_object).norm() + \
    // original_pos + prob_distance * disp;
    Vector3d desired_position = local_displace_dis * Vector3d(1.0,0.0,0.0) + original_pos + Vector3d(0.0, local_prob_distance, 0.0);

    torque = compute_position_cmd_torques(robot, link, pos_in_link, desired_position, kp, i_error, ki, kv);
    if((desired_position - current_position).norm() < POSITION_ERROR_TOLER)
    {
      delay_counter++;
      if (delay_counter > 20000)
      {
        cout << link << " finished positioning!!" << endl;
        state = 8;
        delay_counter = 0; 
      }
    }
  }

  else if (state == 8) // has reached the second intermediate point
  {
    torque = compute_force_cmd_torques(robot, link, pos_in_link, - 2 * local_displace_dis * Vector3d(1.0,0.0,0.0) + original_pos + Vector3d(0.0, prob_distance, 0.0), local_surface_force);
    Vector3d temp_finger_velocity = Vector3d::Zero();
    robot->linearVelocity(temp_finger_velocity, link, pos_in_link);
    velocity_record.pop_front();
    velocity_record.push_back(temp_finger_velocity.norm());
    if(velocity_record[1] < STATIC_THRESHOLD)
    {
      static_counter++;
    }
    if ((velocity_record[1]/velocity_record[0] < CONTACT_COEFFICIENT && velocity_record[0] > MIN_COLLISION_V) || static_counter > 20000)
    {
      state = 9;
      velocity_record.pop_front();
      velocity_record.push_back(0.0);
      velocity_record.pop_front();
      velocity_record.push_back(0.0);
      cout << link <<" contact"<<endl;
      // cout<< "the previous velocity is: " << velocity_record[0] << endl;
      // cout << "the current velocity is: " << velocity_record[1] << endl;

      contact_points.push_back(current_position); 
      // cout << contact_points[100] << "test" << endl; 
      static_counter = 0;  
    }
  }
  else if (state == 9)
  {
    torque = compute_joint_cmd_torques_one_finger(robot, pregrasp_angles, index);
    if (torque.norm() < torque_threshold)
    {
      delay_counter++;
      if (delay_counter > 20000)
      {
        state = 10;
        delay_counter = 0;
      }
    }
  }

  else if(state == 10) 
  {
    /*    Vector3d disp = Vector3d(0.0, 0.0, 0.0);
      Vector3d origin_disp = (original_pos - CoM_of_object) / (original_pos - CoM_of_object).norm();
      disp[0] = origin_disp[1]/sqrt(pow(origin_disp[0], 2) + pow(origin_disp[1], 2));
      disp[1] = - origin_disp[0]/sqrt(pow(origin_disp[0], 2) + pow(origin_disp[1], 2));
      disp = - disp;
    */

    // Vector3d desired_position = displacement_dis*(original_pos - CoM_of_object) / (original_pos - CoM_of_object).norm() + 
    // original_pos + prob_distance * disp;
    Vector3d desired_position = local_displace_dis * Vector3d(1.0,0.0,0.0) + original_pos + Vector3d(0.0, -local_prob_distance, 0.0);

    torque = compute_position_cmd_torques(robot, link, pos_in_link, desired_position, kp, i_error, ki, kv);
    if((desired_position - current_position).norm() < POSITION_ERROR_TOLER)
    {
      delay_counter++;
      if (delay_counter > 20000)
      {
        cout << link << " finished positioning!!" << endl;
        state = 11;
        delay_counter = 0; 
      }
    }
  }

  else if (state == 11) // has reached the fourth intermediate point
  {
    torque = compute_force_cmd_torques(robot, link, pos_in_link, -2 * local_displace_dis * Vector3d(1.0,0.0,0.0) + original_pos + Vector3d(0.0, -prob_distance, 0.0), local_surface_force);
    // cout << torque << endl;
    Vector3d temp_finger_velocity = Vector3d::Zero();
    robot->linearVelocity(temp_finger_velocity, link, pos_in_link);
    velocity_record.pop_front();
    velocity_record.push_back(temp_finger_velocity.norm());
    if(velocity_record[1] < STATIC_THRESHOLD)
    {
      static_counter++;
    }
    if ((velocity_record[1]/velocity_record[0] < CONTACT_COEFFICIENT && velocity_record[0] > MIN_COLLISION_V) || static_counter > 20000)
    {
      state = 12;
      velocity_record.pop_front();
      velocity_record.push_back(0.0);
      velocity_record.pop_front();
      velocity_record.push_back(0.0);
      cout << link <<" contact "<<endl;
      // cout<< "the previous velocity is: " << velocity_record[0] << endl;
      // cout << "the current velocity is: " << velocity_record[1] << endl;

      contact_points.push_back(current_position); 
      // cout << contact_points[100] << "test" << endl; 
      static_counter = 0;  
    }
  }
    else if (state == 12)
  {
    torque = compute_joint_cmd_torques_one_finger(robot, pregrasp_angles, index);
    if (torque.norm() < torque_threshold)
    {
      delay_counter++;
      if (delay_counter > 20000)
      {
        state = 13;
        delay_counter = 0;
      }
    }
  }

  else if (state == 13) // go back to the original contact position
  {
    torque = compute_position_cmd_torques(robot, link, pos_in_link, original_pos, kp, i_error, ki, kv);
    if((original_pos - current_position).norm() < POSITION_ERROR_TOLER)
    {
      cout << link << " arrived the original position" << endl;
      state = 14;
    }
  }

  else if (state == 14)  // compute the normal
  {
    cout << "I am computing the normal for "<< link << endl; 
    // cout << "contact_points" << endl;
    Matrix3d coefficient_matrix = Matrix3d::Zero();
    Vector3d mean_position = Vector3d(0.0, 0.0, 0.0);
    vector<Vector3d> centralized_position = contact_points;
    for (unsigned int j = 0; j < contact_points.size(); j++ )
    {
      mean_position += contact_points[j];
      // cout << contact_points[j] <<endl<< endl;
    }
    mean_position /= double(contact_points.size());
    for (unsigned int j = 0; j < contact_points.size(); j++)
    {
      centralized_position[j] -= mean_position;
    }
    for (unsigned int j = 0; j < contact_points.size(); j++)
    {
      coefficient_matrix += centralized_position[j] * centralized_position[j].transpose();
    }
    EigenSolver<Matrix3d> solver(coefficient_matrix);
    Matrix3d eigen_matrix = solver.eigenvectors().real();
    Vector3d eigen_values = solver.eigenvalues().real();
    int min_index = 999;
    double min_value = 999;
    for (int j = 0; j < 3; j++)
    {
      if (eigen_values[j] < min_value)
      {
        min_value = eigen_values[j];
        min_index = j;
      }
    }
    normal = eigen_matrix.real().col(min_index);
    // the following code chose which direction should the normal choose
    // it's the direction position to the CoM
    Vector3d temp = CoM_of_object - original_pos;
    if (link == "finger0-link3")
    {
      if (normal[0] < 0)
      {
        normal = -normal;
      }
      
    }
    else if(normal[0] > 0)
    {
      normal = -normal;
    }

    cout << "Here is the normal for finger " << link << endl << normal << endl << endl;
    state = 15;
  }
  else if (state == 15) // maintain the original contact position
  {
    torque = compute_position_cmd_torques(robot, link, pos_in_link, original_pos, 100.0, i_error);
  }
  return torque;
}

bool check_2_finger_grasp(vector<Vector3d> contact_points,vector<Vector3d> normals, double friction_coefficient)
{
  double alpha;
  alpha = atan(friction_coefficient);
  Vector3d connect_vector = Vector3d::Zero();
  contact_points.push_back(contact_points[0]);
  normals.push_back(normals[0]);
  int flag = 0;
  for(int i = 0; i < NUM_OF_FINGERS_USED; i++)
  {
    flag = 0;
    connect_vector = contact_points[i+1] - contact_points[i];
    if(normals[i].dot(connect_vector)/(normals[i].norm() * connect_vector.norm()) > cos(alpha));
    {
      flag++;
    }
    if(-normals[i+1].dot(connect_vector)/(normals[i+1].norm() * connect_vector.norm()) < cos(alpha))
    {
      flag++;
    }
    if (flag == 2)
    {
      return true;
    }
  }
  return false;
}

bool check_3_finger_grasp(vector<Vector3d> contact_points,vector<Vector3d> normals, double friction_coefficient)
{
  return true;
}

VectorXd driver_to_sai2 (double q[MAX_DOF])
{
  VectorXd _q = VectorXd::Zero(MAX_DOF + 6);
  _q.block(0,0,6,1) = VectorXd::Zero(6);
  _q[6] = -q[12];
  _q[7] = q[13];
  _q[8] = -q[14];
  _q[9] = -q[15];
  _q[10] = q[0];
  _q[11] = q[1];
  _q[12] = q[2];
  _q[13] = q[3];
  _q[14] = q[4];
  _q[15] = q[5];
  _q[16] = q[6];
  _q[17] = q[7];
  _q[18] = q[8];
  _q[19] = q[9];
  _q[20] = q[10];
  _q[21] = q[11];
  return _q;
}
void sai2_to_driver(VectorXd _q, double q[MAX_DOF])
{
  q[12] = -_q[6];
  q[13] = _q[7];
  q[14] = -_q[8];
  q[15] = -_q[9];
  q[0] = _q[10];
  q[1] = _q[11];
  q[2] = _q[12];
  q[3] = _q[13];
  q[4] = _q[14];
  q[5] = _q[15];
  q[6] = _q[16];
  q[7] = _q[17];
  q[8] = _q[18];
  q[9] = _q[19];
  q[10] = _q[20];
  q[11] = _q[21];
}
VectorXd make_contact(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position, double force_requeired, int& contact_flag, int& static_counter, deque<double>& velocity_record)
{
  VectorXd temp_finger_command_torques = compute_force_cmd_torques(robot, link, pos_in_link, desired_position, force_requeired);
  // VectorXd temp_finger_command_torques = compute_velocity_cmd_torques(robot, link, pos_in_link, desired_position,65.0, 2.5, 0.01);
  // cout << temp_finger_command_torques << endl << endl;

  // finger_command_torques[i].block(6+4*i,0,4,1) = temp_finger_command_torques[i].block(6 + 4 * i, 0, 4, 1);
  Vector3d temp_finger_velocity = Vector3d::Zero();
  robot->linearVelocity(temp_finger_velocity, link, pos_in_link);
  velocity_record.pop_front();
  velocity_record.push_back(temp_finger_velocity.norm());
  if (velocity_record[1] < STATIC_THRESHOLD)
  {
    static_counter += 1;
  }
  if ((velocity_record[1]/velocity_record[0] < CONTACT_COEFFICIENT && velocity_record[0] > MIN_COLLISION_V)||static_counter > 20000)
  {
    static_counter = 0;
    cout << link <<" contact"<<endl;
    cout << "the previous velocity is: " << velocity_record[0] << endl;
    cout << "the current velocity is: " << velocity_record[1] << endl;
    contact_flag = 1;
  }

  return temp_finger_command_torques; 
}
void send_to_redis(CDatabaseRedisClient& redis_cli, vector<Vector3d>contact_points, vector<Vector3d> normals)
{
  contact_points = hemisphere_contact_points(contact_points, normals);
  // cout << "here are the contact points" << endl;
  // cout << contact_points[0] << endl << endl;
  // cout << contact_points[1] << endl << endl;
  // cout << contact_points[2] << endl << endl;
  redis_cli.setEigenMatrixDerived(CONTACT_POSITION_1_KEY, contact_points[0]);
  redis_cli.setEigenMatrixDerived(CONTACT_POSITION_2_KEY, contact_points[1]);
  redis_cli.setEigenMatrixDerived(CONTACT_POSITION_3_KEY, contact_points[2]);
  redis_cli.setEigenMatrixDerived(CONTACT_NORMAL_1_KEY, normals[0]);
  redis_cli.setEigenMatrixDerived(CONTACT_NORMAL_2_KEY, normals[1]);
  redis_cli.setEigenMatrixDerived(CONTACT_NORMAL_3_KEY, normals[2]);
  redis_cli.setEigenMatrixDerived(COM_KEY, CoM_of_object);
  redis_cli.setCommandIs(MASS_KEY, to_string(mass));
}
void set_zero(vector<Vector3d>& variables)
{
  for (int i = 0; i < variables.size(); i++)
  {
    variables[i] = Vector3d::Zero();
  }
}

VectorXd bounce_back(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d optimal_position, Vector3d optimal_force, double kp, double kv)
{
  int dof = robot->dof();
  Vector3d position;
  Vector3d velocity;
  Vector3d force;
  VectorXd torque;
  MatrixXd Jv = MatrixXd::Zero(3,dof);
  robot->Jv(Jv, link, pos_in_link);
  robot->position(position, link, pos_in_link);
  robot->linearVelocity(velocity, link, pos_in_link);
  Vector3d position_error = optimal_position - position;
  position_error = position_error - position_error.dot(optimal_force) * optimal_force / pow(optimal_force.norm(),2);
  force = kp * position_error - kv * velocity;
  torque = Jv.transpose() * force;
  return torque;
}
VectorXd compute_velocity_cmd_torques(Sai2Model::Sai2Model* robot, string link, Vector3d pos_in_link, Vector3d desired_position, double kp, double kv, double desired_velocity)
{
  int dof = robot->dof();
  Vector3d position;
  Vector3d velocity;
  Vector3d force;
  VectorXd torque;
  MatrixXd Jv = MatrixXd::Zero(3,dof);
  robot->Jv(Jv, link, pos_in_link);
  robot->position(position, link, pos_in_link);
  robot->linearVelocity(velocity, link, pos_in_link);
  Vector3d xddot = kp / kv * (desired_position - position);
  double v = min(1.0, desired_velocity / xddot.norm());
  cout << desired_velocity / xddot.norm() << endl;
  force = -kv * (velocity - v * xddot);
  torque = Jv.transpose() * force;
  return torque;
}
vector<Vector3d> hemisphere_contact_points(vector<Vector3d> contact_points, vector<Vector3d> normals)
{
  vector<Vector3d> actual_contact_points = contact_points;
  for( int i = 0; i < contact_points.size(); i++)
  {
    actual_contact_points[i] += normals[i] * radius;
  }
  return actual_contact_points;
}
VectorXd compute_joint_cmd_torques(Sai2Model::Sai2Model* robot, VectorXd desired_joint_angles)
{
  double kp = 1;
  double kv = 0.05;
  VectorXd q = robot->_q;
  VectorXd dq = robot->_dq;
  int dof = robot->dof();
  VectorXd torques = VectorXd::Zero(dof);
  for(int i = 0; i < dof; i++)
  {
    torques[i] = kp * (desired_joint_angles[i] - q[i]) - kv * dq[i];
  }
  return torques;
}
VectorXd compute_joint_cmd_torques_one_finger(Sai2Model::Sai2Model* robot, VectorXd desired_joint_angles, int index)
{
  int dof = robot->dof();
  VectorXd torques = compute_joint_cmd_torques(robot, desired_joint_angles);
  for( int i = 0; i < dof; i++)
  {
    if((i < index * 4 + 10) && (i >= index * 4 + 6))
    {
      continue;
    }
    else
    {
      torques[i] = 0;
    }
  }
  return torques;
}
vector<VectorXd> block_torque(vector<VectorXd> temp_torques)
{
  vector<VectorXd> finger_command_torques;
  for(int i = 0; i < NUM_OF_FINGERS_IN_MODEL; i++)
  {
    finger_command_torques.push_back(VectorXd::Zero(ROBOT_DOF));
  }
  finger_command_torques[0].block(6,0,4,1) = temp_torques[0].block(6,0,4,1);
  finger_command_torques[1].block(10,0,4,1) = temp_torques[1].block(10,0,4,1);
  finger_command_torques[2].block(14,0,4,1) = temp_torques[2].block(14,0,4,1);
  finger_command_torques[3].block(18,0,4,1) = temp_torques[3].block(18,0,4,1);
  return finger_command_torques;
}