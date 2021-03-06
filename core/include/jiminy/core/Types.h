///////////////////////////////////////////////////////////////////////////////
/// \brief    Contains types used in the optimal module.
///////////////////////////////////////////////////////////////////////////////

#ifndef WDC_OPTIMAL_TYPES_H
#define WDC_OPTIMAL_TYPES_H

#include <unordered_map>
#include <vector>

#include "pinocchio/container/aligned-vector.hpp"
#include "pinocchio/spatial/force.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/variant.hpp>
#include <boost/functional/hash.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/tag.hpp>


namespace jiminy
{
    // ******************* General definitions *******************

    // "Standard" types
    using bool_t = bool;
    using char_t = char;
    using float32_t = float;
    using float64_t = double;

    // Eigen types
    using matrixN_t = Eigen::Matrix<float64_t, Eigen::Dynamic, Eigen::Dynamic>;
    using matrix3_t = Eigen::Matrix<float64_t, 3, 3>;
    using vectorN_t = Eigen::Matrix<float64_t, Eigen::Dynamic, 1>;
    using vector3_t = Eigen::Matrix<float64_t, 3, 1>;
    using vector6_t = Eigen::Matrix<float64_t, 6, 1>;
    using rowN_t = Eigen::Matrix<float64_t, 1, Eigen::Dynamic>;

    using constBlockXpr = Eigen::Block<matrixN_t const, Eigen::Dynamic, Eigen::Dynamic>;
    using blockXpr = Eigen::Block<matrixN_t, Eigen::Dynamic, Eigen::Dynamic>;

    using quaternion_t = Eigen::Quaternion<float64_t>;

    // Pinocchio types
    using forceVector_t = pinocchio::container::aligned_vector<pinocchio::Force>;

    // *************** Constant of the universe ******************

    // Define some constant of the universe
    float64_t const INF = std::numeric_limits<float64_t>::infinity();
    float64_t const EPS = std::numeric_limits<float64_t>::epsilon();

    // *************** Jiminy-specific definitions ***************

    // Error codes
    enum class result_t : int32_t
    {
        SUCCESS = 1,
        ERROR_GENERIC = -1,
        ERROR_BAD_INPUT = -2,
        ERROR_INIT_FAILED = -3
    };

    /* Ground profile signature.
       Note that it is impossible to use function pointer since it does not support functors. */
    using heatMapFunctor_t = std::function<std::pair<float64_t, vector3_t>(vector3_t const & /*pos*/)>;

    // Flexible joints
    struct flexibleJointData_t
    {
        std::string jointName;
        vectorN_t stiffness;
        vectorN_t damping;

        flexibleJointData_t(void) :
        jointName(),
        stiffness(),
        damping()
        {
            // Empty.
        };

        flexibleJointData_t(std::string const & jointNameIn,
                            vectorN_t   const & stiffnessIn,
                            vectorN_t   const & dampingIn) :
        jointName(jointNameIn),
        stiffness(stiffnessIn),
        damping(dampingIn)
        {
            // Empty.
        };

        inline bool_t operator==(flexibleJointData_t const & other) const
        {
            return (this->jointName == other.jointName
                 && this->stiffness == other.stiffness
                 && this->damping == other.damping);
        };
    };

    using flexibilityConfig_t =  std::vector<flexibleJointData_t>;

    // Configuration/option holder
    using configField_t = boost::make_recursive_variant<
        bool_t, uint32_t, int32_t, float64_t, std::string, vectorN_t, matrixN_t,
        std::vector<std::string>, std::vector<vectorN_t>, std::vector<matrixN_t>,
        flexibilityConfig_t, heatMapFunctor_t,
        std::unordered_map<std::string, boost::recursive_variant_> >::type;

    using configHolder_t = std::unordered_map<std::string, configField_t>;

    // Sensor data holder
    struct sensorDataTypePair_t {
        // Disable the copy of the class
        sensorDataTypePair_t(sensorDataTypePair_t const & sensorDataPairIn) = delete;
        sensorDataTypePair_t & operator = (sensorDataTypePair_t const & other) = delete;

        sensorDataTypePair_t(std::string const & nameIn,
                             int32_t     const & idIn,
                             vectorN_t   const * valueIn) :
        name(nameIn),
        id(idIn),
        value(valueIn)
        {
            // Empty.
        };

        ~sensorDataTypePair_t(void) = default;

        sensorDataTypePair_t(sensorDataTypePair_t && other) :
        name(other.name),
        id(other.id),
        value(other.value)
        {
            // Empty.
        };

        std::string name;
        int32_t id;
        vectorN_t const * value;
    };

    using namespace boost::multi_index;
    struct IndexByName {};
    struct IndexById {};
    using sensorDataTypeMap_t = multi_index_container<
        sensorDataTypePair_t,
        indexed_by<
            ordered_unique<
                tag<IndexById>,
                member<sensorDataTypePair_t, int32_t, &sensorDataTypePair_t::id>,
                std::less<int32_t> // Ordering by ascending order
            >,
            hashed_unique<
                tag<IndexByName>,
                member<sensorDataTypePair_t, std::string, &sensorDataTypePair_t::name>
            >
        >
    >;

    using sensorsDataMap_t = std::unordered_map<std::string, sensorDataTypeMap_t>;
}

#endif  // WDC_OPTIMAL_TYPES_H
