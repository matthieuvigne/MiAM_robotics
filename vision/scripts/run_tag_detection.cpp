// System libraries
#include <algorithm>
#include <cassert>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>

// Third party libraries
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

// ________________________________________ CAMERA _________________________________________________

struct Marker {
  double radius;
  double theta_rad;
  double height;
}; // struct Marker
typedef std::vector<Marker> MarkerList;

class Camera {

  /*
   *  [NOTE] This camera model is a standard pinhole model which assumes no distortion.
   */

  public:
    Camera() = delete;
    Camera(int width, int height);
    Camera(int width, int height, double fx, double fy, double cx, double cy);
    typedef std::shared_ptr<Camera> Ptr;
    typedef std::unique_ptr<Camera> UniquePtr;
    virtual ~Camera();
  
  public:

    // Getters
    inline double fx() const { return fx_; }
    inline double fy() const { return fy_; }
    inline double cx() const { return cx_; }
    inline double cy() const { return cy_; }
    inline int width() const { return width_; }
    inline int height() const { return height_; }
    inline Eigen::Affine3d TRC() const { return TRC_; }
  
    // Setters
    inline double& fx() { return fx_; }
    inline double& fy() { return fy_; }
    inline double& cx() { return cx_; }
    inline double& cy() { return cy_; }
    inline Eigen::Affine3d& TRC() { return TRC_; }
  
    // Get markers
    bool get_markers(MarkerList* markers);
  
  protected:
  
    // Get the camera matrix
    cv::Mat get_camera_matrix() const;
  
    // Detect the markers within the image
    int detect_markers(
      cv::Mat const& img,
      std::vector<Eigen::Affine3d>* TRM_ptr) const;
  
  protected:
  
    // Extrinsic parameters
    Eigen::Affine3d TRC_ = Eigen::Affine3d::Identity();
    
    // Intrinsic parameters
    double fx_ = NAN;     // x axis focal length [px/m]
    double fy_ = NAN;     // y axis focal length [px/m]
    double cx_ = NAN;     // x axis optical center coordinate [px
    double cy_ = NAN;     // y axis optical center coordinate [px]
    
    // Image resolution
    int width_;           // image width [px]
    int height_;          // image height [px]
    
    // Marker detection
    cv::VideoCapture capture_;
    typedef cv::Ptr<cv::aruco::DetectorParameters> CvParamsPtr;
    typedef cv::Ptr<cv::aruco::Dictionary> CvDicoPtr;
    CvParamsPtr cv_params_ptr_;
    CvDicoPtr cv_dico_ptr_;
    
}; // class Camera

Camera::Camera(int width, int height)
: width_(width), height_(height),
  fx_(NAN), fy_(NAN),
  cx_(width/2.), cy_(height/2.)
{
  // Check the image resolution
  assert( (width_>0) );
  assert( (height_>0) );
  
  // Initialize the ARUCO detector parameters
  cv_dico_ptr_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  cv_params_ptr_ = cv::aruco::DetectorParameters::create();
  
  return;
}

Camera::~Camera()
{
  if(capture_.isOpened())
    capture_.release();
  return;
}

Camera::Camera(int width, int height, double fx, double fy, double cx, double cy)
: Camera(width, height)
{
  // Set focal lenghts
  assert( (fx>0.) && (fy>0.) );
  fx_ = fx;
  fy_ = fy;
  
  // Set optical center coordinates
  assert( (cx>0.) && (cy>0.) );
  cx_ = cx;
  cy_ = cy;
  return;
}

cv::Mat Camera::get_camera_matrix() const
{
  cv::Mat K = cv::Mat::eye(3,3,CV_64FC1);
  K.at<double>(0,0) = fx_;
  K.at<double>(1,1) = fy_;
  K.at<double>(0,2) = cx_;
  K.at<double>(1,2) = cy_;
  return K;
}

int Camera::detect_markers(
  cv::Mat const& img,
  std::vector<Eigen::Affine3d>* TRM_ptr) const
{
  // Detect the markers
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<std::vector<cv::Point2f>> rejected_candidates;
  cv::aruco::detectMarkers(img, cv_dico_ptr_, marker_corners, marker_ids,
    cv_params_ptr_, rejected_candidates);
  
  // Remove all detected markers other than ID47
  assert( marker_ids.size() == marker_corners.size() );
  std::vector<int>::iterator marker_it = marker_ids.begin();
  std::vector<std::vector<cv::Point2f>>::iterator corner_it = marker_corners.begin();
  while( marker_it != marker_ids.end() )
  {
    if(*marker_it != 47u)
    {
      marker_it = marker_ids.erase(marker_it);
      corner_it = marker_corners.erase(corner_it);
    } else {
      marker_it += 1;
      corner_it += 1;
    }
  }

  // Get the number of detected markers
  int const num_markers = static_cast<int>(marker_ids.size());
  if(num_markers == 0) return 0;
  
  // Estimate the pose of each marker
  std::vector<cv::Vec3d> rvecs, tvecs;
  double constexpr marker_length = 5e-2;
  cv::Mat const K = this->get_camera_matrix();
  cv::Mat dist_coeffs = cv::Mat(4,1,CV_64FC1, cv::Scalar::all(0));
  cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_length, K, dist_coeffs, rvecs, tvecs);
  assert( (rvecs.size()==num_markers) && (tvecs.size()==num_markers) );
  
  // Return the pose of each marker in the robot's reference frame
  assert( TRM_ptr != NULL );
  std::vector<Eigen::Affine3d>& TRM = *TRM_ptr;
  TRM.clear();
  TRM.resize(num_markers);
  for(int marker_idx=0; marker_idx<num_markers; marker_idx+=1)
  {
    cv::Vec3d const& rvec = rvecs[marker_idx];
    double const angle_rad = cv::norm(rvec);
    Eigen::Vector3d const axis = Eigen::Vector3d(rvec[0],rvec[1],rvec[2]).normalized();
    Eigen::AngleAxisd const rCM(angle_rad,axis);
    Eigen::Map<Eigen::Vector3d const> tCM(tvecs[marker_idx].val);
    Eigen::Affine3d TCM(Eigen::Translation3d(tCM)*rCM);
    TRM[marker_idx] = TRC_ * TCM;
  }
  return num_markers;
}

bool Camera::get_markers(MarkerList* markers_ptr)
{
  // Open the video capture channel if required
  // Enforce the image resolution of the video capture channel
  if(!capture_.isOpened())
  {
    capture_.open("/dev/video0");
    if(!capture_.isOpened()) return false;
    capture_.set(cv::CAP_PROP_FRAME_WIDTH,width_);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT,height_);
  }
  
  // Get the current image
  cv::Mat img;
  capture_ >> img;
  assert( img.cols == width_ );
  assert( img.rows == height_ );
  std::vector<Eigen::Affine3d> TRMs;
  int const num_markers = detect_markers(img,&TRMs);
  
  // Convert into marker polar coordinates
  assert( markers_ptr != NULL );
  MarkerList& markers = *markers_ptr;
  markers.resize(num_markers);
  for(int marker_idx=0; marker_idx<num_markers; marker_idx+=1)
  {
    Eigen::Vector3d const pRM = TRMs[marker_idx].translation();
    double const radius = pRM.norm();
    double const theta_rad = std::atan2(pRM.y(),pRM.x());
    markers[marker_idx] = Marker{radius,theta_rad,pRM.z()};
  }
  
  return true;
}

// _________________________________________________________________________________________________

class Breaker {

  public:
    Breaker();
    ~Breaker();
    bool is_break_requested() const;

  private:
    static void handler(int signal);
    static bool is_instantiated_;
    static bool is_sigint_raised_;
    void (*previous_handler_)(int);
}; // class Breaker

Breaker::Breaker()
: previous_handler_(signal(SIGINT, &Breaker::handler))
{
  assert(!is_instantiated_);
  is_instantiated_ = true;
  is_sigint_raised_ = false;
}

Breaker::~Breaker() {
  assert(signal(SIGINT,previous_handler_) == &Breaker::handler);
  is_instantiated_ = false;
}

bool Breaker::is_break_requested() const {
  return is_sigint_raised_;
}

void Breaker::handler(int signal) {
  assert(signal == SIGINT);
  is_sigint_raised_ = true;
}

bool Breaker::is_instantiated_ = false;
bool Breaker::is_sigint_raised_ = false;

// ______________________________________ MAIN _____________________________________________________

int main(int argc, char* argv[])
{
  // Instantiate the camera
  int const width = 640;
  int const height = 480;
  int const fx = 330.;
  int const fy = 330.;
  int const cx = fx/2.;
  int const cy = fy/2.;
  Camera::Ptr camera_ptr(new Camera(width,height,fx,fy,cx,cy));
  camera_ptr->TRC() = Eigen::AngleAxisd(M_PI_2,Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(M_PI_2,Eigen::Vector3d::UnitZ());
  
  // Run the detection loop
  Breaker breaker;
  while(true)
  {
    MarkerList detected_markers;
    camera_ptr->get_markers(&detected_markers);
    if(!detected_markers.empty())
    {
      std::cout << "Detected " << detected_markers.size() << " markers." << std::endl;
      for(Marker const& marker : detected_markers)
      {
        std::stringstream msg;
        msg << "- marker detected at " << marker.radius << " m" << std::endl;
        std::cout << msg.str();
      }
    } else {
      std::cout << "No marker has been detected!" << std::endl;
    }
    
    // Abort if break has been requested
    if(breaker.is_break_requested()) break;
    cv::waitKey(100);
  }
  
  return EXIT_SUCCESS;
}

// _________________________________________________________________________________________________
// -------------------------------------------------------------
// mutex.lock();
// >> DO SOMETHING
// mutex.unlock();
// condition_variable.notify_one();
// -------------------------------------------------------------
// std::unique_lock<std::mutex> locker(mutex);
// condition_variable.wait(locker,[&](){return condition;});
// >> DO SOMETHING
// locker.unlock();
// -------------------------------------------------------------
