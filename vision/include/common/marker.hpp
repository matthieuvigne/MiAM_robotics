#ifndef COMMON_TAGS_HPP
#define COMMON_TAGS_HPP

#include <map>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <common/time.hpp>

namespace common {

//--------------------------------------------------------------------------------------------------
// Marker class
//--------------------------------------------------------------------------------------------------

class Marker {

  public:
    typedef uint8_t Id;
    enum class Status { INVALID, MEASURED, ESTIMATED };
    enum class Family {
      UNKNOWN,
      CENTRAL_MARKER,
      ROCK_SAMPLE,
      TREASURE_RED_SAMPLE,
      TREASURE_GREEN_SAMPLE,
      TREASURE_BLUE_SAMPLE,
      PURPLE_TEAM_ROBOT,
      YELLOW_TEAM_ROBOT,
      PURPLE_TEAM_MARKER,
      YELLOW_TEAM_MARKER
    }; // enum class Family

    typedef std::vector<Marker> MarkerList;
    typedef std::map<Id,Marker> MarkerIdToEstimate;
    typedef std::multimap<int64_t,Marker,std::less<int64_t>> MarkerEstimates;
    POINTER_TYPEDEF(Marker);

  public:
    Marker();
    Marker(Id id);

  virtual ~Marker() = default;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:

    // Getters
    inline Id getId() const;
    inline Status getStatus() const;
    inline Family getMarkerFamily() const;
    inline int64_t getTimestampNanoseconds() const;
    inline double getTimestampSeconds() const;
    inline Eigen::Affine3d const* getTCM() const;
    inline Eigen::Matrix<double,6,6> const* getCovTCM() const;
    inline Eigen::Affine3d const* getTWM() const;
    inline Eigen::Matrix<double,6,6> const* getCovTWM() const;
    inline double getSizeLength() const;
    bool isUnique() const;
  
    // Measurement and estimates
    void addMeasurement(
      int64_t timestamp_ns,
      std::vector<cv::Point2f> const& corners,
      Eigen::Affine3d const& TCM,
      Eigen::Matrix<double,6,6> const& cov_TCM);
    void addEstimate(
      Eigen::Affine3d const& TWM,
      Eigen::Matrix<double,6,6> const& cov_TWM);
    void estimateFromCameraPose(
      Eigen::Affine3d const& TWC,
      Eigen::Matrix<double,6,6> const& cov_TWC);
  
    // Serialization
    bool serialize(
      std::vector<char>::iterator it_begin,
      std::vector<char>::iterator it_end) const;
    bool deserialize(
      std::vector<char>::const_iterator it_begin,
      std::vector<char>::const_iterator it_end);
    std::string print() const;

  public:

    // Static functions
    static Family getMarkerFamily(Id id);
    static Id sampleMarkerId(Family marker_family);
    static bool serialize(MarkerEstimates const& estimates, std::vector<char>* message);
    static bool deserialize(std::vector<char> const& message, MarkerEstimates* estimates);    
    static int constexpr MESSAGE_SIZE_BYTES =  1 * sizeof(Id)       + /*Marker id*/
                                               1 * sizeof(Family)   + /*Marker family*/
                                               1 * sizeof(int64_t)  + /*timestamp*/
                                               7 * sizeof(double);    /*Pose*/
  private:

    // Status
    Status status_;
    Id id_;
    Family family_;

    // Measurements
    int64_t timestamp_ns_;
    std::vector<cv::Point2f> corners_;
    std::shared_ptr<Eigen::Affine3d> TCM_;
    std::shared_ptr<Eigen::Matrix<double,6,6>> cov_TCM_;

    // Estimate
    std::shared_ptr<Eigen::Affine3d> TWM_;
    std::shared_ptr<Eigen::Matrix<double,6,6>> cov_TWM_;

}; // class Marker

//--------------------------------------------------------------------------------------------------
// Typedefs
//--------------------------------------------------------------------------------------------------

typedef Marker::Id MarkerId;
typedef Marker::MarkerList MarkerList;
typedef Marker::MarkerIdToEstimate MarkerIdToEstimate;
typedef Marker::MarkerEstimates MarkerEstimates;

//--------------------------------------------------------------------------------------------------
// Inline functions
//--------------------------------------------------------------------------------------------------

Marker::Id Marker::getId() const
{
  return id_;
}

//--------------------------------------------------------------------------------------------------

Marker::Status Marker::getStatus() const
{
  return status_;
}

//--------------------------------------------------------------------------------------------------

Marker::Family Marker::getMarkerFamily() const
{
  return family_;
}

//--------------------------------------------------------------------------------------------------

int64_t Marker::getTimestampNanoseconds() const 
{
  return timestamp_ns_;
};

//--------------------------------------------------------------------------------------------------

double Marker::getTimestampSeconds() const
{
  int64_t const seconds = timestamp_ns_ % static_cast<int64_t>(1e9);
  int64_t const nanoseconds = timestamp_ns_ - seconds;
  return static_cast<double>(seconds) + static_cast<double>(nanoseconds)/1e9; 
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d const* Marker::getTCM() const
{
  Eigen::Affine3d const* ptr = TCM_ ? TCM_.get() : NULL;
  return ptr;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> const* Marker::getCovTCM() const
{
  Eigen::Matrix<double,6,6> const* ptr = cov_TCM_ ? cov_TCM_.get() : NULL;
  return ptr;
}

//--------------------------------------------------------------------------------------------------

Eigen::Affine3d const* Marker::getTWM() const
{
  Eigen::Affine3d const* ptr = TWM_ ? TWM_.get() : NULL;
  return ptr;
}

//--------------------------------------------------------------------------------------------------

Eigen::Matrix<double,6,6> const* Marker::getCovTWM() const
{
  Eigen::Matrix<double,6,6> const* ptr = cov_TWM_ ? cov_TWM_.get() : NULL;
  return ptr;
}

//--------------------------------------------------------------------------------------------------

double Marker::getSizeLength() const
{
  return (id_==42) ? 0.10 : 0.05;
}

//--------------------------------------------------------------------------------------------------

} // namespace common

#endif // COMMON_TAGS_HPP
