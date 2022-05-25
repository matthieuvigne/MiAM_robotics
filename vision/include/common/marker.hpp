#ifndef COMMON_TAGS_HPP
#define COMMON_TAGS_HPP

#include <map>
#include <memory>
#include <vector>

#include <eigen3/Eigen/Dense>

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
    typedef std::shared_ptr<Marker> Ptr;
    typedef std::shared_ptr<Marker const> ConstPtr;
    typedef std::unique_ptr<Marker> UniquePtr;

  public:
    Marker();
    Marker(Id id);
    Marker(Marker const& marker);

  virtual ~Marker() = default;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:

    // Getters
    inline Id getId() const;
    bool isUnique() const;
    static bool isUnique(Id const marker_id);
    static bool isUnique(Family const family);
    inline bool isCentralMarker() const;
    inline Status getStatus() const;
    inline Family getMarkerFamily() const;
    inline double getSizeLength() const;
    inline Eigen::Vector3d const& getRuM() const;

    // Timestamp
    inline int64_t getTimestampNanoseconds() const;
    inline double getTimestampSeconds() const;

    // Poses
    inline Eigen::Affine3d const* getTCM() const;
    inline Eigen::Matrix<double,6,6> const* getCovTCM() const;
    inline Eigen::Affine3d const* getTWM() const;
    inline Eigen::Matrix<double,6,6> const* getCovTWM() const;

    // Measurement and estimates
    void addMeasurement(
      int64_t timestamp_ns,
      Eigen::Vector3d const& RuM,
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
    static bool serialize(MarkerList const& markers, std::vector<char>* message);
    static bool deserialize(std::vector<char> const& message, MarkerList* markers);
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
    std::shared_ptr<Eigen::Affine3d> TCM_;
    std::shared_ptr<Eigen::Matrix<double,6,6>> cov_TCM_;
    Eigen::Vector3d RuM_;

    // Estimate
    std::shared_ptr<Eigen::Affine3d> TWM_;
    std::shared_ptr<Eigen::Matrix<double,6,6>> cov_TWM_;

}; // class Marker

//--------------------------------------------------------------------------------------------------
// Typedefs
//--------------------------------------------------------------------------------------------------

typedef Marker::Id MarkerId;
typedef Marker::MarkerList MarkerList;
typedef std::vector<Marker::UniquePtr> MarkerPtrList;

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

Eigen::Vector3d const& Marker::getRuM() const
{
  return RuM_;
}

//--------------------------------------------------------------------------------------------------

bool Marker::isCentralMarker() const
{
  return (static_cast<int>(id_) == 42);
}

//--------------------------------------------------------------------------------------------------

} // namespace common

#endif // COMMON_TAGS_HPP
