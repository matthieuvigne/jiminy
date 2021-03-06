#ifndef JIMINY_ENGINE_H
#define JIMINY_ENGINE_H

#include <tuple>
#include <string>
#include <functional>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/energy.hpp"

#include "jiminy/core/Utilities.h"
#include "jiminy/core/Model.h"
#include "jiminy/core/TelemetrySender.h"
#include "jiminy/core/Types.h"

#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen_algebra.hpp>


namespace jiminy
{
    std::string const ENGINE_OBJECT_NAME("HighLevelController");

    extern float64_t const MIN_SIMULATION_TIMESTEP;
    extern float64_t const MAX_SIMULATION_TIMESTEP;

    using namespace boost::numeric::odeint;

    class AbstractController;
    class TelemetryData;
    class TelemetryRecorder;

    class explicit_euler
    {
    public:
        using state_type = vectorN_t;
        using deriv_type = vectorN_t;
        using value_type = float64_t;
        using time_type = float64_t;
        using order_type = unsigned short;

        using stepper_category = controlled_stepper_tag;

        static order_type order(void)
        {
            return 1;
        }

        template<class System>
        controlled_step_result try_step(System       system,
                                        state_type & x,
                                        deriv_type & dxdt,
                                        time_type  & t,
                                        time_type  & dt) const
        {
            t += dt;
            system(x, dxdt, t);
            x += dt * dxdt;
            return controlled_step_result::success;
        }
    };

    struct stepperState_t
    {
    public:
        stepperState_t(void) :
        iter(0),
        t(0.0),
        dt(0.0),
        t_err(0.0),
        x(),
        dxdt(),
        u(),
        uCommand(),
        uMotor(),
        uInternal(),
        fExternal(),
        nx_(0),
        nq_(0),
        nv_(0),
        isInitialized_(false)
        {
            // Empty.
        }

        void initialize(Model & model)
        {
            initialize(model, vectorN_t::Zero(model.nx()), MIN_SIMULATION_TIMESTEP);
        }

        void initialize(Model           & model,
                        vectorN_t const & xInit,
                        float64_t const & dt_init)
        {
            // Extract some information from the model
            nx_ = model.nx();
            nq_ = model.nq();
            nv_ = model.nv();

            // Initialize the ode stepper state buffers
            iter = 0;
            t = 0.0;
            dt = dt_init;
            x = xInit;

            dxdt = vectorN_t::Zero(nx_);
            computePositionDerivative(model.pncModel_, q(), v(), qDot());

            fExternal = forceVector_t(model.pncModel_.joints.size(),
                                      pinocchio::Force::Zero());
            uInternal = vectorN_t::Zero(nv_);
            uCommand = vectorN_t::Zero(model.getMotorsNames().size());
            uMotor = vectorN_t::Zero(model.getMotorsNames().size());
            u = vectorN_t::Zero(nv_);

            // Set the initialization flag
            isInitialized_ = true;
        }

        bool_t const & getIsInitialized(void) const
        {
            return isInitialized_;
        }

        Eigen::Ref<vectorN_t> q(void)
        {
            return x.head(nq_);
        }

        Eigen::Ref<vectorN_t> v(void)
        {
            return x.tail(nv_);
        }

        Eigen::Ref<vectorN_t> qDot(void)
        {
            return dxdt.head(nq_);
        }

        Eigen::Ref<vectorN_t> a(void)
        {
            return dxdt.tail(nv_);
        }

    public:
        uint32_t iter;
        float64_t t;
        float64_t dt;
        float64_t t_err;            ///< Sum of error internal buffer used for Kahan algorithm
        vectorN_t x;
        vectorN_t dxdt;
        vectorN_t u;
        vectorN_t uCommand;
        vectorN_t uMotor;
        vectorN_t uInternal;
        forceVector_t fExternal;

    private:
        uint32_t nx_;
        uint32_t nq_;
        uint32_t nv_;

        bool_t isInitialized_;
    };

    class Engine
    {
    public:
        // Impossible to use function pointer since it does not support functors
        using forceFunctor_t = std::function<vector3_t(float64_t const & /*t*/,
                                                       vectorN_t const & /*x*/)>;
        using callbackFunctor_t =  std::function<bool_t(float64_t const & /*t*/,
                                                        vectorN_t const & /*x*/)>;

    protected:
        using rungeKuttaStepper_t = runge_kutta_dopri5<vectorN_t, float64_t, vectorN_t, float64_t, vector_space_algebra>;
        using stepper_t = boost::variant<result_of::make_controlled<rungeKuttaStepper_t>::type, explicit_euler>;

    public:
        // Disable the copy of the class
        Engine(Engine const & engine) = delete;
        Engine & operator = (Engine const & other) = delete;

    public:
        configHolder_t getDefaultContactOptions()
        {
            configHolder_t config;
            config["frictionViscous"] = 0.8;
            config["frictionDry"] = 1.0;
            config["dryFrictionVelEps"] = 1.0e-2;
            config["stiffness"] = 1.0e6;
            config["damping"] = 2.0e3;
            config["transitionEps"] = 1.0e-3;

            return config;
        };

        struct contactOptions_t
        {
            float64_t const frictionViscous;
            float64_t const frictionDry;
            float64_t const dryFrictionVelEps;
            float64_t const stiffness;
            float64_t const damping;
            float64_t const transitionEps;

            contactOptions_t(configHolder_t const & options) :
            frictionViscous(boost::get<float64_t>(options.at("frictionViscous"))),
            frictionDry(boost::get<float64_t>(options.at("frictionDry"))),
            dryFrictionVelEps(boost::get<float64_t>(options.at("dryFrictionVelEps"))),
            stiffness(boost::get<float64_t>(options.at("stiffness"))),
            damping(boost::get<float64_t>(options.at("damping"))),
            transitionEps(boost::get<float64_t>(options.at("transitionEps")))
            {
                // Empty.
            }
        };

        configHolder_t getDefaultJointOptions()
        {
            configHolder_t config;
            config["boundStiffness"] = 1.0e5;
            config["boundDamping"] = 1.0e4;
            config["boundTransitionEps"] = 1.0e-2; // about 0.55 degrees

            return config;
        };

        struct jointOptions_t
        {
            float64_t const boundStiffness;
            float64_t const boundDamping;
            float64_t const boundTransitionEps;

            jointOptions_t(configHolder_t const & options) :
            boundStiffness(boost::get<float64_t>(options.at("boundStiffness"))),
            boundDamping(boost::get<float64_t>(options.at("boundDamping"))),
            boundTransitionEps(boost::get<float64_t>(options.at("boundTransitionEps")))
            {
                // Empty.
            }
        };

        configHolder_t getDefaultWorldOptions()
        {
            configHolder_t config;
            config["gravity"] = (vectorN_t(6) << 0.0, 0.0, -9.81, 0.0, 0.0, 0.0).finished();
            config["groundProfile"] = heatMapFunctor_t(
                [](vector3_t const & pos) -> std::pair <float64_t, vector3_t>
                {
                    return {0.0, (vector3_t() << 0.0, 0.0, 1.0).finished()};
                });

            return config;
        };

        struct worldOptions_t
        {
            vectorN_t const gravity;
            heatMapFunctor_t const groundProfile;

            worldOptions_t(configHolder_t const & options) :
            gravity(boost::get<vectorN_t>(options.at("gravity"))),
            groundProfile(boost::get<heatMapFunctor_t>(options.at("groundProfile")))
            {
                // Empty.
            }
        };

        configHolder_t getDefaultStepperOptions()
        {
            configHolder_t config;
            config["verbose"] = false;
            config["randomSeed"] = 0U;
            config["odeSolver"] = std::string("runge_kutta_dopri5"); // ["runge_kutta_dopri5", "explicit_euler"]
            config["tolAbs"] = 1.0e-5;
            config["tolRel"] = 1.0e-4;
            config["dtMax"] = 1.0e-3;
            config["iterMax"] = 100000; // -1: infinity
            config["sensorsUpdatePeriod"] = 0.0;
            config["controllerUpdatePeriod"] = 0.0;
            config["logInternalStepperSteps"] = false;

            return config;
        };

        struct stepperOptions_t
        {
            bool_t      const verbose;
            uint32_t    const randomSeed;
            std::string const odeSolver;
            float64_t   const tolAbs;
            float64_t   const tolRel;
            float64_t   const dtMax;
            int32_t     const iterMax;
            float64_t   const sensorsUpdatePeriod;
            float64_t   const controllerUpdatePeriod;
            bool_t      const logInternalStepperSteps;

            stepperOptions_t(configHolder_t const & options) :
            verbose(boost::get<bool_t>(options.at("verbose"))),
            randomSeed(boost::get<uint32_t>(options.at("randomSeed"))),
            odeSolver(boost::get<std::string>(options.at("odeSolver"))),
            tolAbs(boost::get<float64_t>(options.at("tolAbs"))),
            tolRel(boost::get<float64_t>(options.at("tolRel"))),
            dtMax(boost::get<float64_t>(options.at("dtMax"))),
            iterMax(boost::get<int32_t>(options.at("iterMax"))),
            sensorsUpdatePeriod(boost::get<float64_t>(options.at("sensorsUpdatePeriod"))),
            controllerUpdatePeriod(boost::get<float64_t>(options.at("controllerUpdatePeriod"))),
            logInternalStepperSteps(boost::get<bool_t>(options.at("logInternalStepperSteps")))
            {
                // Empty.
            }
        };

        configHolder_t getDefaultTelemetryOptions()
        {
            configHolder_t config;
            config["enableConfiguration"] = true;
            config["enableVelocity"] = true;
            config["enableAcceleration"] = true;
            config["enableTorque"] = true;
            config["enableEnergy"] = true;
            return config;
        };

        struct telemetryOptions_t
        {
            bool_t const enableConfiguration;
            bool_t const enableVelocity;
            bool_t const enableAcceleration;
            bool_t const enableTorque;
            bool_t const enableEnergy;

            telemetryOptions_t(configHolder_t const & options) :
            enableConfiguration(boost::get<bool_t>(options.at("enableConfiguration"))),
            enableVelocity(boost::get<bool_t>(options.at("enableVelocity"))),
            enableAcceleration(boost::get<bool_t>(options.at("enableAcceleration"))),
            enableTorque(boost::get<bool_t>(options.at("enableTorque"))),
            enableEnergy(boost::get<bool_t>(options.at("enableEnergy")))
            {
                // Empty.
            }
        };

        configHolder_t getDefaultOptions()
        {
            configHolder_t config;
            config["telemetry"] = getDefaultTelemetryOptions();
            config["stepper"] = getDefaultStepperOptions();
            config["world"] = getDefaultWorldOptions();
            config["joints"] = getDefaultJointOptions();
            config["contacts"] = getDefaultContactOptions();

            return config;
        };

        struct engineOptions_t
        {
            telemetryOptions_t const telemetry;
            stepperOptions_t   const stepper;
            worldOptions_t     const world;
            jointOptions_t     const joints;
            contactOptions_t   const contacts;

            engineOptions_t(configHolder_t const & options) :
            telemetry(boost::get<configHolder_t>(options.at("telemetry"))),
            stepper(boost::get<configHolder_t>(options.at("stepper"))),
            world(boost::get<configHolder_t>(options.at("world"))),
            joints(boost::get<configHolder_t>(options.at("joints"))),
            contacts(boost::get<configHolder_t>(options.at("contacts")))
            {
                // Empty.
            }
        };

    public:
        Engine(void);
        ~Engine(void);

        result_t initialize(std::shared_ptr<Model>              const & model,
                            std::shared_ptr<AbstractController> const & controller,
                            callbackFunctor_t    callbackFct);

        /// \brief Reset engine.
        ///
        /// \details This function resets the engine, the model and the controller.
        ///          This method is made to be called in between simulations, to allow
        ///          registering of new variables to log, and reset the random number
        ///          generator.
        ///
        /// \param[in] resetDynamicForceRegister Whether or not to register the external force profiles applied
        ///                                      during the simulation.
        void reset(bool_t const & resetDynamicForceRegister = false);

        /// \brief Reset the engine and compute initial state.
        ///
        /// \details This function reset the engine, the model and the controller, and update internal data
        ///          to match the given initial state.
        ///
        /// \param[in] xInit Initial state.
        /// \param[in] isStateTheoretical Specify if the initial state is associated with the current or theoretical model
        /// \param[in] resetRandomNumbers Whether or not to reset the random number generator.
        /// \param[in] resetDynamicForceRegister Whether or not to register the external force profiles applied
        ///                                      during the simulation.
        result_t start(vectorN_t const & xInit,
                       bool_t    const & isStateTheoretical = false,
                       bool_t    const & resetRandomNumbers = false,
                       bool_t    const & resetDynamicForceRegister = false);

        /// \brief Integrate system from current state for a duration equal to stepSize
        ///
        /// \details This function performs a single 'integration step', in the sense that only
        ///          the endpoint is added to the log. The integrator object is allowed to perform
        ///          multiple steps inside of this interval.
        ///          One may specify a negative timestep to use the default update value.
        ///
        /// \param[in] stepSize Duration for which to integrate ; set to negative value to use default update value.
        result_t step(float64_t stepSize = -1);

        /// \brief Stop the simulation.
        ///
        /// \details It releases the lock on the model and the telemetry, so that
        ///          it is possible again to update the model (for example to update
        ///          the options, add or remove sensors...) and to register new
        ///          variables or forces.
        void stop(void);

        /// \brief Run a simulation of duration tEnd, starting at xInit.
        ///
        /// \param[in] tEnd End time, i.e. amount of time to simulate.
        /// \param[in] xInit Initial state, i.e. state at t=0.
        /// \param[in] isStateTheoretical Specify if the initial state is associated with the current or theoretical model
        result_t simulate(float64_t const & tEnd,
                          vectorN_t const & xInit,
                          bool_t    const & isStateTheoretical = false);

        result_t registerForceImpulse(std::string const & frameName,
                                      float64_t   const & t,
                                      float64_t   const & dt,
                                      vector3_t   const & F);
        result_t registerForceProfile(std::string      const & frameName,
                                      forceFunctor_t           forceFct);

        configHolder_t const & getOptions(void) const;
        result_t setOptions(configHolder_t const & engineOptions);
        bool_t getIsInitialized(void) const;
        bool_t getIsTelemetryConfigured(void) const;
        Model & getModel(void) const;
        AbstractController & getController(void) const;
        stepperState_t const & getStepperState(void) const;
        std::vector<vectorN_t> const & getContactForces(void) const;

        void getLogDataRaw(std::vector<std::string>             & header,
                           std::vector<float64_t>               & timestamps,
                           std::vector<std::vector<int32_t> >   & intData,
                           std::vector<std::vector<float32_t> > & floatData);

        /// \brief Get the full logged content.
        ///
        /// \param[out] header      Header, vector of field names.
        /// \param[out] logData     Corresponding data in the log file.
        void getLogData(std::vector<std::string> & header,
                        matrixN_t                & logData);

        /// \brief Get the value of a single logged variable.
        ///
        /// \param[in] fieldName    Full name of the variable to get
        /// \param[in] header       Header, vector of field names.
        /// \param[in] logData      Corresponding data in the log file.
        ///
        /// \return Vector of values for fieldName. If fieldName is not in the header list, this vector will be empty.
        static vectorN_t getLogFieldValue(std::string              const & fieldName,
                                          std::vector<std::string>       & header,
                                          matrixN_t                      & logData);

        result_t writeLogTxt(std::string const & filename);
        result_t writeLogBinary(std::string const & filename);

        static result_t parseLogBinaryRaw(std::string                          const & filename,
                                          std::vector<std::string>                   & header,
                                          std::vector<float64_t>                     & timestamps,
                                          std::vector<std::vector<int32_t> >         & intData,
                                          std::vector<std::vector<float32_t> >       & floatData);
        static result_t parseLogBinary(std::string              const & filename,
                                       std::vector<std::string>       & header,
                                       matrixN_t                      & logData);

    protected:
        result_t configureTelemetry(void);
        void updateTelemetry(void);

        vector6_t computeFrameForceOnParentJoint(int32_t   const & frameId,
                                                 vector3_t const & fExtInWorld) const;
        vector3_t computeContactDynamics(int32_t const & frameId) const;
        void computeForwardKinematics(Eigen::Ref<vectorN_t const> q,
                                      Eigen::Ref<vectorN_t const> v,
                                      Eigen::Ref<vectorN_t const> a);
        void computeCommand(float64_t                   const & t,
                            Eigen::Ref<vectorN_t const>         q,
                            Eigen::Ref<vectorN_t const>         v,
                            vectorN_t                         & u);
        void computeExternalForces(float64_t     const & t,
                                   vectorN_t     const & x,
                                   forceVector_t       & fext);
        void computeInternalDynamics(float64_t                   const & t,
                                     Eigen::Ref<vectorN_t const>         q,
                                     Eigen::Ref<vectorN_t const>         v,
                                     vectorN_t                         & u);
        void computeSystemDynamics(float64_t const & tIn,
                                   vectorN_t const & xIn,
                                   vectorN_t       & dxdtIn);

    private:
        void reset(bool_t const & resetRandomNumbers,
                   bool_t const & resetDynamicForceRegister);

        template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl,
                 typename ConfigVectorType, typename TangentVectorType>
        inline Scalar
        kineticEnergy(pinocchio::ModelTpl<Scalar,Options,JointCollectionTpl> const & model,
                      pinocchio::DataTpl<Scalar,Options,JointCollectionTpl>        & data,
                      Eigen::MatrixBase<ConfigVectorType>                    const & q,
                      Eigen::MatrixBase<TangentVectorType>                   const & v,
                      bool_t                                                 const & update_kinematics);
        template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl,
                 typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2,
                 typename ForceDerived>
        inline const typename pinocchio::DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
        rnea(pinocchio::ModelTpl<Scalar,Options,JointCollectionTpl> const & model,
             pinocchio::DataTpl<Scalar,Options,JointCollectionTpl>        & data,
             Eigen::MatrixBase<ConfigVectorType>                    const & q,
             Eigen::MatrixBase<TangentVectorType1>                  const & v,
             Eigen::MatrixBase<TangentVectorType2>                  const & a,
             pinocchio::container::aligned_vector<ForceDerived>     const & fext);
        template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl,
                 typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2,
                 typename ForceDerived>
        inline const typename pinocchio::DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
        aba(pinocchio::ModelTpl<Scalar,Options,JointCollectionTpl> const & model,
            pinocchio::DataTpl<Scalar,Options,JointCollectionTpl>        & data,
            Eigen::MatrixBase<ConfigVectorType>                    const & q,
            Eigen::MatrixBase<TangentVectorType1>                  const & v,
            Eigen::MatrixBase<TangentVectorType2>                  const & tau,
            pinocchio::container::aligned_vector<ForceDerived>     const & fext);

    public:
        std::unique_ptr<engineOptions_t const> engineOptions_;

    protected:
        bool_t isInitialized_;
        bool_t isTelemetryConfigured_;
        std::shared_ptr<Model> model_;
        std::shared_ptr<AbstractController> controller_;
        configHolder_t engineOptionsHolder_;
        callbackFunctor_t callbackFct_;

    private:
        std::unique_ptr<MutexLocal::LockGuardLocal> lockModel_;
        TelemetrySender telemetrySender_;
        std::shared_ptr<TelemetryData> telemetryData_;
        std::unique_ptr<TelemetryRecorder> telemetryRecorder_;
        stepper_t stepper_;
        float64_t stepperUpdatePeriod_;
        stepperState_t stepperState_;       ///< Internal buffer with the state for the integration loop
        stepperState_t stepperStateLast_;   ///< Internal state for the integration loop at the end of the previous iteration
        std::map<float64_t, std::tuple<std::string, float64_t, vector3_t> > forcesImpulse_; // Note that one MUST use an ordered map wrt. the application time
        std::map<float64_t, std::tuple<std::string, float64_t, vector3_t> >::const_iterator forceImpulseNextIt_;
        std::vector<std::pair<std::string, std::tuple<int32_t, forceFunctor_t> > > forcesProfile_;
    };
}

#endif //end of JIMINY_ENGINE_H
