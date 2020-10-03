#include "pinocchio/algorithm/joint-configuration.hpp"


#include "jiminy/core/robot/Robot.h"
#include "jiminy/core/control/AbstractController.h"
#include "jiminy/core/engine/System.h"


namespace jiminy
{
    // ====================================================
    // ================== forceProfile_t ==================
    // ====================================================

    forceProfile_t::forceProfile_t(std::string           const & frameNameIn,
                                   int32_t               const & frameIdxIn,
                                   forceProfileFunctor_t const & forceFctIn) :
    frameName(frameNameIn),
    frameIdx(frameIdxIn),
    forceFct(forceFctIn)
    {
        // Empty on purpose
    }

    // ====================================================
    // ================== forceImpulse_t ==================
    // ====================================================

    forceImpulse_t::forceImpulse_t(std::string      const & frameNameIn,
                                   int32_t          const & frameIdxIn,
                                   float64_t        const & tIn,
                                   float64_t        const & dtIn,
                                   pinocchio::Force const & FIn) :
    frameName(frameNameIn),
    frameIdx(frameIdxIn),
    t(tIn),
    dt(dtIn),
    F(FIn)
    {
        // Empty on purpose
    }

    // ====================================================
    // ================== forceCoupling_t =================
    // ====================================================

    forceCoupling_t::forceCoupling_t(std::string            const & systemName1In,
                                     int32_t                const & systemIdx1In,
                                     std::string            const & systemName2In,
                                     int32_t                const & systemIdx2In,
                                     std::string            const & frameName1In,
                                     int32_t                const & frameIdx1In,
                                     std::string            const & frameName2In,
                                     int32_t                const & frameIdx2In,
                                     forceCouplingFunctor_t const & forceFctIn) :
    systemName1(systemName1In),
    systemIdx1(systemIdx1In),
    systemName2(systemName2In),
    systemIdx2(systemIdx2In),
    frameName1(frameName1In),
    frameIdx1(frameIdx1In),
    frameName2(frameName2In),
    frameIdx2(frameIdx2In),
    forceFct(forceFctIn)
    {
        // Empty on purpose.
    }

    // ====================================================
    // ================== systemHolder_t ==================
    // ====================================================

    systemHolder_t::systemHolder_t(std::string const & systemNameIn,
                                   std::shared_ptr<Robot> robotIn,
                                   std::shared_ptr<AbstractController> controllerIn,
                                   callbackFunctor_t callbackFctIn) :
    name(systemNameIn),
    robot(std::move(robotIn)),
    controller(std::move(controllerIn)),
    callbackFct(std::move(callbackFctIn))
    {
        // Empty on purpose
    }

    systemHolder_t::systemHolder_t(void) :
    systemHolder_t("", nullptr, nullptr,
    [](float64_t const & t,
       vectorN_t const & q,
       vectorN_t const & v) -> bool_t
    {
        return false;
    })
    {
        // Empty on purpose.
    }

    // ===============================================
    // ================ systemState_t ================
    // ===============================================

    systemState_t::systemState_t(void) :
    q(),
    v(),
    a(),
    u(),
    uCommand(),
    uMotor(),
    uInternal(),
    fExternal(),
    isInitialized_(false)
    {
        // Empty on purpose.
    }

    hresult_t systemState_t::initialize(Robot const & robot)
    {
        if (!robot.getIsInitialized())
        {
            std::cout << "Error - systemState_t::initialize - Robot not initialized." << std::endl;
            return hresult_t::ERROR_INIT_FAILED;
        }

        q = pinocchio::neutral(robot.pncModel_);
        v = vectorN_t::Zero(robot.nv());
        a = vectorN_t::Zero(robot.nv());
        u = vectorN_t::Zero(robot.nv());
        uInternal = vectorN_t::Zero(robot.nv());
        uCommand = vectorN_t::Zero(robot.getMotorsNames().size());
        uMotor = vectorN_t::Zero(robot.getMotorsNames().size());
        fExternal = forceVector_t(robot.pncModel_.joints.size(),
                                  pinocchio::Force::Zero());
        isInitialized_ = true;

        return hresult_t::SUCCESS;
    }

    bool_t const & systemState_t::getIsInitialized(void) const
    {
        return isInitialized_;
    }

    // ===============================================
    // ============== systemDataHolder_t =============
    // ===============================================

    systemDataHolder_t::systemDataHolder_t(void) :
    robotLock(nullptr),
    forcesProfile(),
    forcesImpulse(),
    forcesImpulseBreaks(),
    forcesImpulseBreakNextIt(),
    forcesImpulseActive(),
    positionFieldnames(),
    velocityFieldnames(),
    accelerationFieldnames(),
    motorEffortFieldnames(),
    energyFieldname(),
    state(),
    statePrev()
    {
        // Empty on purpose
    }

    systemDataHolder_t::systemDataHolder_t(systemDataHolder_t && other):
    robotLock(std::move(other.robotLock)),
    forcesProfile(std::move(other.forcesProfile)),
    forcesImpulse(std::move(other.forcesImpulse)),
    forcesImpulseBreaks(std::move(other.forcesImpulseBreaks)),
    forcesImpulseBreakNextIt(std::move(other.forcesImpulseBreakNextIt)),
    forcesImpulseActive(std::move(other.forcesImpulseActive)),
    positionFieldnames(std::move(other.positionFieldnames)),
    velocityFieldnames(std::move(other.velocityFieldnames)),
    accelerationFieldnames(std::move(other.accelerationFieldnames)),
    motorEffortFieldnames(std::move(other.motorEffortFieldnames)),
    energyFieldname(std::move(other.energyFieldname)),
    state(std::move(other.state)),
    statePrev(std::move(other.statePrev))
    {
        // Empty on purpose
    }

    systemDataHolder_t & systemDataHolder_t::operator=(systemDataHolder_t && other)
    {
        robotLock = std::move(other.robotLock);
        forcesProfile = std::move(other.forcesProfile);
        forcesImpulse = std::move(other.forcesImpulse);
        forcesImpulseBreaks = std::move(other.forcesImpulseBreaks);
        forcesImpulseBreakNextIt = std::move(other.forcesImpulseBreakNextIt);
        forcesImpulseActive = std::move(other.forcesImpulseActive);
        positionFieldnames = std::move(other.positionFieldnames);
        velocityFieldnames = std::move(other.velocityFieldnames);
        accelerationFieldnames = std::move(other.accelerationFieldnames);
        motorEffortFieldnames = std::move(other.motorEffortFieldnames);
        energyFieldname = std::move(other.energyFieldname);
        state = std::move(other.state);
        statePrev = std::move(other.statePrev);
        return *this;
    }
}
