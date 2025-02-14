#include "pure_pursuit.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include <vector>
#include <utility>
#include <iostream>
#include <tf2/utils.h>
#include <algorithm>

using namespace std;

class PurePursuit : public rclcpp::Node {

  public:
    PurePursuit(): Node("pure_pursuit") {

      waypoints = {{0.4107271211582440, -0.0608933713017566},
    {0.8215009710665580, -0.12147499627094700},
    {1.2323195577355900, -0.18175890183253200},
    {1.6431806401773200, -0.24175894891238100},
    {2.054082060403290, -0.30148916443547200},
    {2.4650216604250600, -0.36096332632810100},
    {2.8759972822541600, -0.4201954615152630},
    {3.2870067679021400, -0.4791994309228090},
    {3.69804795938053, -0.5379890954765910},
    {4.109118698700870, -0.596578565101143},
    {4.520216827874720, -0.6549815347232220},
    {4.93134027191317, -0.7132121142673620},
    {5.342486706828650, -0.7712840816598740},
    {5.753654057632260, -0.8292114638257060},
    {6.164840166335540, -0.8870081216907580},
    {6.576042957949570, -0.9446879161808640},
    {6.987260108486820, -1.0022648742210000},
    {7.398489542958370, -1.0597528567370400},
    {7.809729103375750, -1.1171657246548500},
    {8.220976714750060, -1.1745173389002900},
    {8.632231049088430, -1.2318157504302800},
    {9.04349185739216, -1.2890639472288600},
    {9.454758724663480, -1.3462645852818200},
    {9.866031401903730, -1.4034204035744900},
    {10.27730955711470, -1.460533975093130},
    {10.688592941297700, -1.5176080388230900},
    {11.099881139454900, -1.5746452507501400},
    {11.511173902587700, -1.6316482668601100},
    {11.922470898697900, -1.688619743138780},
    {12.333771795787200, -1.745562418571490},
    {12.745076344857000, -1.8024790321436000},
    {13.156384130909400, -1.8593720738417700},
    {13.567694904945900, -1.9162443656509500},
    {13.979008334968200, -1.9730985635568900},
    {14.390324171977600, -2.029937240545840},
    {14.801642000976400, -2.086763218602740},
    {15.212961572965800, -2.1435790707137700},
    {15.624282555947700, -2.2003874528647700},
    {16.035604700923300, -2.257191187040630},
    {16.446927509895400, -2.313992763228040},
    {16.85825089986440, -2.3707950034118700},
    {17.269574372833, -2.427600480578420},
    {17.680897762802, -2.4844119337129900},
    {18.092220654773700, -2.5412320188013900},
    {18.50354271674980, -2.598063391829430},
    {18.914863782731200, -2.654908791782430},
    {19.326183354720600, -2.7117706256475200},
    {19.737501515717600, -2.768649474421620},
    {20.14881826572210, -2.8255449231069200},
    {20.560133521734600, -2.8824563077070000},
    {20.97144744975430, -2.9393833792231700},
    {21.38275996678140, -2.9963255566585500},
    {21.794071155815700, -3.0532822590162200},
    {22.205381016857000, -3.110253237297550},
    {22.616689549905500, -3.16723782750607},
    {23.027996920960200, -3.224235614644},
    {23.43930296402200, -3.2812461007140100},
    {23.850607762090400, -3.3382688707183200},
    {24.2619113981651, -3.3953034266595900},
    {24.673213955245500, -3.452349187540940},
    {25.084515267332600, -3.5094058213641200}, {25.49581550042550, -3.5664727471322600}, {25.90711465452410, -3.6235494668480000}, {26.318412729628500, -3.680635648513150}, {26.7297097257387, -3.737730711130780}, {27.14100580885380, -3.7948341567035900}, {27.55230081297460, -3.8519455702337600}, {27.9635949041003, -3.909064370724440}, {28.374888082230900, -3.9661902261773900}, {28.78618026436680, -4.0233225555957000}, {29.197471616507200, -4.0804609439816}, {29.60876213865200, -4.137604810338200}, {30.020051747801700, -4.194753822667290}, {30.43134060995540, -4.251907399971950}, {30.842628642113500, -4.3090650442548700}, {31.253915927275600, -4.366226340518240}, {31.665202548441300, -4.423390873764320}, {32.07648842261090, -4.480557979996610}, {32.48777354978460, -4.53772732721693}, {32.89905809596140, -4.594898417427920}, {33.310341978141700, -4.652070669632700}, {33.7216252793251, -4.709243751833050}, {34.13290799951160, -4.766417166031600}, {34.544190138701300, -4.823590331231500}, {34.95547177989360, -4.8807628324349500}, {35.36675292308860, -4.937934171644610}, {35.7780335682862, -4.995103933862720}, {36.18931371548650, -5.052271538092340}, {36.600593530688600, -5.109436569335750}, {37.011872847893300, -5.166598612595130}, {37.423151833099800, -5.223757086873610}, {37.834430486308100, -5.280911494173830}, {38.245708807518200, -5.338061419498020}, {38.656986796730000, -5.395206447848410}, {39.06826453694320, -5.4523459152285600}, {39.479542028157600, -5.509479572639760}, {39.89081935337310, -5.5666067560856}, {40.30209642958980, -5.623727050568290}, {40.71337333980740, -5.6808399580904700}, {41.12465008402590, -5.737945063654390}, {41.53592666224530, -5.795042118261390}, {41.94720324046470, -5.852131370910100}, {42.35847981868400, -5.909213070599220}, {42.76975639690340, -5.9662877983256300}, {43.181032975122800, -6.023355720088430}, {43.592309719341300, -6.080417250885420}, {44.003586546559400, -6.137472722714820}, {44.41486362277610, -6.1945225505744100}, {44.8261408649919, -6.251566983462850}, {45.23741843920600, -6.30860651937748}, {45.648696345418200, -6.365641407316970}, {46.05997466662830, -6.422672062279100}, {46.47125331983660, -6.479698733262560}, {46.88253247104220, -6.5367219182646500}, {47.29381212024520, -6.593741866284050}, {47.705092350445000, -6.650758909318980}, {48.116373078642200, -6.7077735453668000}, {48.52765455383540, -6.764786023426160}, {48.938936610025500, -6.821796675495280}, {49.35021941321160, -6.87880591657195}, {49.761503046393200, -6.935814078654400}, {50.17278742657090, -6.992821493740840}, {50.58407263674410, -7.049828576829050}, {50.99535884291200, -7.1068355769177}
    ,{51.40664596207500, -7.163842992004140}, {51.81793399423310, -7.220851071087030}, {52.22922318838500, -7.277860146164590}, {52.640513378531600, -7.3348706322346000}, {53.05180464767240, -7.391882944294840}, {53.46309716180660, -7.448897248344430}, {53.874390837934500, -7.505914125380270}, {54.285685842055300, -7.562933741401440}, {54.6969820911695, -7.619956594405310}, {55.10827966827660, -7.676982933390520}, {55.5195786563761, -7.734013090355320}, {55.93087905546810, -7.79104756329704}, {56.342180948552100, -7.848086601214340}, {56.75348441862760, -7.905130536105440}, {57.164789382695200, -7.962179782968140}, {57.57609592375430, -8.019234673800650}, {57.9874042078041, -8.076295540601180}, {58.398714151845000, -8.133362798367550}, {58.8100258388766, -8.19043677909794}, {59.22133926889890, -8.247517814790620}, {59.632654607911000, -8.304606237443760}, {60.043971772913200, -8.36170246205519}, {60.455290846905400, -8.418806820623120}, {60.86661191288680, -8.475919645145750}, {61.27793497085760, -8.533041267621330}, {61.689260020817700, -8.590172103047630}, {62.10058722876630, -8.647312483422890}, {62.511916594703300, -8.704462740745300}, {62.923248118628800, -8.761623290012680}, {63.334581883542300, -8.81879438022367}, {63.74591755744560, -8.875975513380950}, {64.15725431034320, -8.9331644484965}, {64.56859114624030, -8.990358612584080}, {64.97992698614270, -9.047555598656560}, {65.39126091705530, -9.104752750728170}, {65.802592025983, -9.161947744811310}, {66.21391923393160, -9.219137924920190}, {66.62524154490640, -9.276320884067700}, {67.0365580459122, -9.333494132267150}, {67.4478676579549, -9.390655096532310}, {67.85916955103890, -9.447801286876490}, {68.27046256317040, -9.504930296312560}, {68.6817458643538, -9.562039551854290}, {69.09301837559500, -9.619126480515420}, {69.50427910089910, -9.676188758308420}, {69.91552721027080, -9.733223729247480}, {70.32676154171610, -9.79022898634545}, {70.73798118224000, -9.847201956616120}, {71.14918513584790, -9.904140233072360}, {71.5603724895445, -9.961041159728370}, {71.97154216433570, -10.017902412596600}, {72.38269324722630, -10.07472141869080}, {72.79382465922210, -10.13149560502470}, {73.20493548732810, -10.188222564611200}, {73.61602481854900, -10.244899807463600}, {74.0270915738906, -10.30152476059570}, {74.43813475735830, -10.358095017020300}, {74.84915810393220, -10.414640456577700}, {75.26018078650960, -10.471295953546400}, {75.67122488297240, -10.528217215093700}, {76.08231247120270, -10.585560114385700}, {76.49346571208170, -10.643480358589800}, {76.90470676649110, -10.70213373787250}, {77.31605762931320, -10.7616761254002}, {77.72754054442900, -10.822263311339600}, {78.13917758972030, -10.884051085857500}, {78.55099092606870, -10.947195156121100}, {78.96300254835660, -11.011851395296500}, {79.37523470046500, -11.078175593550600}, {79.78770954327520, -11.146323541050100}, {80.19932575567800, -11.218124299011900}, {80.58141412901540, -11.336469216024500}, {80.8772240488272, -11.58320822130030}, {81.04039951605630, -11.955187580707300}, {81.0560851871589, -12.369566101335400}, {81.00757651161550, -12.789243064624000}, {80.96463287330670, -13.203039426365900}, {80.92809099075700, -13.615285771398300}, {80.894705083327, -14.027923044339800}, {80.86223582299420, -14.440872561611200}, {80.82845350968450, -14.854035885739000}, {80.7911285263233, -15.267314745248900}, {80.7480311728366, -15.68061086866640}, {80.69700113677920, -16.093826316515400}, {80.639815687645, -16.506882073218600}, {80.58422872696100, -16.919727509046700}, {80.53849373058220, -17.332314401257600}, {80.51086425736300, -17.74459477610800}, {80.50959370015880, -18.15652032785630}, {80.54293561782400, -18.568042999759400}, {80.62254182003710, -18.976141026979800}, {80.82065188341280, -19.32390916688450}, {81.16008304990920, -19.558105694245800}, {81.54907682031180, -19.72501838248570}, {81.92869869584160, -19.889568571361900}, {82.29947232069780, -20.070876785605000}, {82.67284283566390, -20.257605119857700}, {83.05232203495680, -20.420866906625100}, {83.43160909027750, -20.588426576404600}, {83.79213650293720, -20.811568807891500}, {84.05380546835750, -21.107479904162200}, {84.15429469087440, -21.469894394728600}, {84.13976943656510, -21.875740809992100}, {84.0717023306332, -22.299593978943500}, {84.00125313744230, -22.720426628050600}, {83.93731193944240, -23.13523873835960}, {83.87794019900190, -23.545648054217700}, {83.8211997104878, -23.953272402971700}, {83.76515201926810, -24.359729694967800}, {83.70785867071110, -24.76663767455340}, {83.64738079518690, -25.175609023102700}, {83.58166880165840, -25.587108206131400}, {83.50842534541310, -25.99903035489780}, {83.42531930091950, -26.40892167052650}, {83.3301471129636, -26.814444055523200}};

      // for (double i = 0; i <= 10; i += 0.5) {
      //   double y = f(i);             
      //   waypoints.emplace_back(i, y);  
      // }

      num_waypoints = static_cast<int>(waypoints.size());
      goal_point = waypoints[0];

      // SUBSCRIBERS ------------------------------------------------------------------------------------------------------------------
      odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, bind(&PurePursuit::odomCallback, this, placeholders::_1)
      );

      pose_subscriber_ = this->create_subscription<visualization_msgs::msg::Marker>(
            "/f1tenth_car", 10, bind(&PurePursuit::pose_callback, this, placeholders::_1)
      );
      // ------------------------------------------------------------------------------------------------------------------------------

      // TIMERS -----------------------------------------------------------------------------------------------------------------------
      odom_timer_ = this->create_wall_timer(
        chrono::seconds(2), 
        bind(&PurePursuit::odomTimerCallback, this));

      drive_timer_ = this->create_wall_timer(
        chrono::milliseconds(100), 
        bind(&PurePursuit::driveTimerCallback, this));

      pose_timer_ = this->create_wall_timer(
        chrono::seconds(2), 
        bind(&PurePursuit::poseTimerCallback, this));
      
      goal_point_task = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PurePursuit::gptask_callback, this)
      );
      // ------------------------------------------------------------------------------------------------------------------------------

      // PUBLISHERS -------------------------------------------------------------------------------------------------------------------
      drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
      // ------------------------------------------------------------------------------------------------------------------------------    
    }

  private:

    vector<pair<double, double>> waypoints;
    int num_waypoints;

    // Odom Data
    rclcpp::TimerBase::SharedPtr odom_timer_;
    nav_msgs::msg::Odometry::SharedPtr last_odom_msg_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void odomTimerCallback();

    // Drive
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::TimerBase::SharedPtr drive_timer_;
    void driveTimerCallback();

    // Car Pose
    void pose_callback(const visualization_msgs::msg::Marker::SharedPtr msg);
    void poseTimerCallback();
    rclcpp::TimerBase::SharedPtr pose_timer_;
    visualization_msgs::msg::Marker::SharedPtr current_pose_;
    geometry_msgs::msg::Point carPosition;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr pose_subscriber_;

    // Goal Point
    pair<double, double> getGoalPoint(double ld, geometry_msgs::msg::Point carPos);
    double pointDistance(pair<double, double> car, pair<double, double> point);
    pair<double, double> interpolatedPoint(double ld, pair<double, double> p1, pair<double, double> p2, pair<double, double> car);
    double sgn(double num);
    bool checkSolution(double maxX, double minX, double maxY, double minY, pair<double, double> sol);
    double f(double x);
    pair<double, double> goal_point;
    rclcpp::TimerBase::SharedPtr goal_point_task;
    void gptask_callback();
    int last_point = 0;
    double steering_angle;
    double heading;
};

// ODOM SUBSCRIBER CALLBACKS -----------------------------------------------------------------------------------------------------------
void PurePursuit::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  last_odom_msg_ = msg;
  heading = tf2::getYaw(msg->pose.pose.orientation);
}

void PurePursuit::odomTimerCallback() {
  // if (last_odom_msg_ != nullptr) {
  //   RCLCPP_INFO(this->get_logger(), "Odom Data: x=%.2f, y=%.2f", last_odom_msg_->pose.pose.position.x, last_odom_msg_->pose.pose.position.y);
  // } 
  
  // else {
  //   RCLCPP_WARN(this->get_logger(), "No odometry data received yet.");
  // }
}
// ------------------------------------------------------------------------------------------------------------------------------------

// POSE SUBSCRIBER CALLBACKS -----------------------------------------------------------------------------------------------------------
void PurePursuit::pose_callback(const visualization_msgs::msg::Marker::SharedPtr msg) {
  current_pose_ = msg;
}

void PurePursuit::poseTimerCallback() {
  if (current_pose_ != nullptr) {
    //RCLCPP_INFO(this->get_logger(), "Pose Data: x=%.2f, y=%.2f", current_pose_->pose.position.x, current_pose_->pose.position.y);
    carPosition = current_pose_->pose.position;
    //getGoalPoint(.4, carPosition);
  } 
  
  else {
    RCLCPP_WARN(this->get_logger(), "No pose data received yet.");
  }
}
// ------------------------------------------------------------------------------------------------------------------------------------


// DRIVE PUBLISHER --------------------------------------------------------------------------------------------------------------------
void PurePursuit::driveTimerCallback() {
  auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  drive_msg.header.stamp = this->get_clock()->now();

  double target_speed = 0.5; // meters per second

  drive_msg.drive.speed = target_speed;
  drive_msg.drive.steering_angle = steering_angle;
  drive_msg.drive.steering_angle_velocity = 100;

  // Publish the drive command
  drive_publisher_->publish(drive_msg);
}
// ------------------------------------------------------------------------------------------------------------------------------------

// GOAL POINT -------------------------------------------------------------------------------------------------------------------------
void PurePursuit::gptask_callback() {
  double ld = 2.6;

  goal_point = getGoalPoint(ld, carPosition);

  double carLength = 1;
  double dy = goal_point.second-carPosition.y;
  double dx = goal_point.first-carPosition.x;
  double alpha = atan2(dy, dx);
  double diff = alpha-heading;

  double curvature = 2*sin(diff)/ld;

  steering_angle = atan(curvature*carLength); // radians
}

double PurePursuit::pointDistance(pair<double, double> car, pair<double, double> point) {
  double distance = pow((pow((car.first-point.first), 2)+pow((car.second-point.second), 2)), 0.5);
  return distance;
}

double PurePursuit::sgn(double num) {
  if(num >= 0) {
    return 1;
  }

  else {
    return -1;
  }
}

bool PurePursuit::checkSolution(double maxX, double minX, double maxY, double minY, pair<double, double> sol) {
  if(sol.first >= minX && sol.first <= maxX && sol.second >= minY && sol.second <= maxY) {
    return true;
  }

  else {
    return false;
  }
}

pair<double, double> PurePursuit::interpolatedPoint(double ld, pair<double, double> p1, pair<double, double> p2, pair<double, double> car) {

  // Offset the Circle to the origin
  double x1 = p1.first - car.first;
  double x2 = p2.first - car.first;
  double y1 = p1.second - car.second;
  double y2 = p2.second - car.second;

  double dX = x2 - x1; // delta x
  double dY = y2 - y1; // delta y
  double d = pow((pow(dX, 2) + pow(dY, 2)), 0.5); // distance between points
  double D = (x1*y2) - (x2*y1); // Determinant for matrix
  double discriminant = (pow(ld, 2)*pow(d, 2) - pow(D, 2));


  if(discriminant >= 0) { // We will only care about real solutions

    double sol1_x = ((D*dY + sgn(dY)*dX*pow(discriminant, 0.5)))/pow(d, 2);
    double sol1_y = ((-1*D*dX + abs(dY)*pow(discriminant, 0.5)))/pow(d, 2);

    double sol2_x = ((D*dY - sgn(dY)*dX*pow(discriminant, 0.5)))/pow(d, 2);
    double sol2_y = ((-1*D*dX - abs(dY)*pow(discriminant, 0.5)))/pow(d, 2);

    // Translate back to original position
    pair<double, double> sol1 = {sol1_x + car.first, sol1_y + car.second};
    pair<double, double> sol2 = {sol2_x + car.first, sol2_y + car.second};

    // Check if POIs are within line segment
    double minX = min(p1.first, p2.first);
    double maxX = max(p1.first, p2.first);
    double minY = min(p1.second, p2.second);
    double maxY = max(p1.second, p2.second);

    bool sol1_validity = checkSolution(maxX, minX, maxY, minY, sol1);
    bool sol2_validity = checkSolution(maxX, minX, maxY, minY, sol2);

    if(sol1_validity && sol2_validity) {
      // Find which POI is closer to the second point
      double dP2_sol1 = pow((pow(p2.first-sol1.first, 2) + pow(p2.second-sol1.second, 2)), 0.5);
      double dP2_sol2 = pow((pow(p2.first-sol2.first, 2) + pow(p2.second-sol2.second, 2)), 0.5);

      if(dP2_sol1 < dP2_sol2) {
        return sol1;
      }

      else {
        return sol2;
      }
    }

    else if (sol1_validity) {
      return sol1;
    }

    else if (sol2_validity) {
      return sol2;
    }
  } 

  return {0,0}; // No valid interpolated points were found

}

pair<double, double> PurePursuit::getGoalPoint(double ld, geometry_msgs::msg::Point carPos) {

  pair<double, double> goalPoint = goal_point;
  pair<double, double> carPoint = {carPos.x, carPos.y};

  for(int i=last_point; i < num_waypoints; i++) {

    if(ld - pointDistance(carPoint, waypoints[i]) == 0) { // found point exactly lookahead distance away (MAYBE ADD A BUFFER MARGIN)
      goalPoint = waypoints[i];
      last_point = i;
      //RCLCPP_INFO(this->get_logger(), "Goal Point: x=%.2f y=%.2f", goalPoint.first, goalPoint.second);
      return goalPoint;
    }

    else if(ld - pointDistance(carPoint, waypoints[i]) < 0) { // first point beyond lookahead distance
      int j = i-1;
      while(ld - pointDistance(carPoint, waypoints[j]) < 0 && j > 0) { j--; } // find last point within lookahead distance
      goalPoint = interpolatedPoint(ld, waypoints[i], waypoints[j], carPoint);

      if(goalPoint.first != 0 && goalPoint.second != 0) {
        last_point = j;
        //RCLCPP_INFO(this->get_logger(), "Goal Point: x=%.2f y=%.2f", goalPoint.first, goalPoint.second);
        return goalPoint;
      }
    }
  }
  
  return goalPoint;
}

double PurePursuit::f(double x) {
  return -.25*sin(x);
  //return -.3*atan(x);
}
// ------------------------------------------------------------------------------------------------------------------------------------


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<PurePursuit>());
  rclcpp::shutdown();
  return 0;
}
