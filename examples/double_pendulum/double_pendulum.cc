// A simple test case: simulation of a double inverted pendulum.
// There are no contact forces.
// This simulation checks the overall simulator sanity (i.e. conservation of energy) and genericity (working
// with something that is not an exoskeleton).

#include <sys/types.h>
#include <iostream>
#include <string>

#include "jiminy/core/Utilities.h"
#include "jiminy/core/Engine.h"
#include "jiminy/core/BasicMotors.h"
#include "jiminy/core/ControllerFunctor.h"
#include "jiminy/core/Types.h"


using namespace jiminy;

void computeCommand(float64_t        const & t,
                    vectorN_t        const & q,
                    vectorN_t        const & v,
                    sensorsDataMap_t const & sensorsData,
                    vectorN_t              & u)
{
    // No controller: energy should be preserved.
    u.setZero();
}

void internalDynamics(float64_t      const & t,
                      vectorN_t        const & q,
                      vectorN_t        const & v,
                      sensorsDataMap_t const & sensorsData,
                      vectorN_t              & u)
{
    u.setZero();
}

bool_t callback(float64_t const & t,
              vectorN_t const & x)
{
    return true;
}

int main(int argc, char_t * argv[])
{
    // =====================================================================
    // ==================== Extract the user paramaters ====================
    // =====================================================================

    // Set URDF and log output.
    std::string homedir = getUserDirectory();
    std::string urdfPath = homedir + std::string("/wdc_workspace/src/jiminy/data/double_pendulum/double_pendulum.urdf");
    std::string outputDirPath("/tmp/blackbox/");

    // =====================================================================
    // ============ Instantiate and configure the simulation ===============
    // =====================================================================

    // Instantiate timer
    Timer timer;

    timer.tic();

    // Instantiate and configuration the model
    std::vector<std::string> motorJointNames{"SecondPendulumJoint"};

    auto model = std::make_shared<Model>();
    configHolder_t mdlOptions = model->getOptions();
    boost::get<bool_t>(boost::get<configHolder_t>(mdlOptions.at("joints")).at("positionLimitFromUrdf")) = true;
    boost::get<bool_t>(boost::get<configHolder_t>(mdlOptions.at("joints")).at("velocityLimitFromUrdf")) = true;
    model->setOptions(mdlOptions);
    model->initialize(urdfPath, false);
    for (std::string const & jointName : motorJointNames)
    {
        std::shared_ptr<SimpleMotor> motor = std::make_shared<SimpleMotor>(jointName);
        model->attachMotor(motor);
        motor->initialize(jointName);
    }

    // Instantiate and configuration the controller

    auto controller = std::make_shared<ControllerFunctor<decltype(computeCommand),
                                                         decltype(internalDynamics)> >(computeCommand, internalDynamics);
    controller->initialize(model);

    // Instantiate and configuration the engine
    Engine engine;
    configHolder_t simuOptions = engine.getDefaultOptions();
    boost::get<bool_t>(boost::get<configHolder_t>(simuOptions.at("telemetry")).at("enableConfiguration")) = true;
    boost::get<bool_t>(boost::get<configHolder_t>(simuOptions.at("telemetry")).at("enableVelocity")) = true;
    boost::get<bool_t>(boost::get<configHolder_t>(simuOptions.at("telemetry")).at("enableAcceleration")) = true;
    boost::get<bool_t>(boost::get<configHolder_t>(simuOptions.at("telemetry")).at("enableTorque")) = true;
    boost::get<bool_t>(boost::get<configHolder_t>(simuOptions.at("telemetry")).at("enableEnergy")) = true;
    boost::get<vectorN_t>(boost::get<configHolder_t>(simuOptions.at("world")).at("gravity"))(2) = -9.81;
    boost::get<std::string>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("odeSolver")) = std::string("runge_kutta_dopri5");
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("tolRel")) = 1.0e-5;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("tolAbs")) = 1.0e-4;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("dtMax")) = 3.0e-3;
    boost::get<int32_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("iterMax")) = 100000U; // -1 for infinity
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("sensorsUpdatePeriod")) = 1.0e-3;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("controllerUpdatePeriod")) = 1.0e-3;
    boost::get<bool_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("logInternalStepperSteps")) = false;
    boost::get<uint32_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("randomSeed")) = 0U; // Use time(nullptr) for random seed.
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("stiffness")) = 1e6;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("damping")) = 2000.0;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("dryFrictionVelEps")) = 0.01;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("frictionDry")) = 5.0;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("frictionViscous")) = 5.0;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("transitionEps")) = 0.001;
    engine.setOptions(simuOptions);
    engine.initialize(model, controller, callback);

    timer.toc();

    // =====================================================================
    // ======================= Run the simulation ==========================
    // =====================================================================

    // Prepare options
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    x0(1) = 0.1;
    float64_t tf = 3.0;

    // Run simulation
    timer.tic();
    engine.simulate(tf, x0);
    timer.toc();
    std::cout << "Simulation time: " << (timer.dt * 1.0e3) << "ms" << std::endl;

    // Write the log file
    std::vector<std::string> header;
    matrixN_t log;
    engine.getLogData(header, log);
    std::cout << log.rows() << " log points" << std::endl;
    engine.writeLogTxt(outputDirPath + std::string("/log.txt"));
    engine.writeLogBinary(outputDirPath + std::string("/log.data"));

    return 0;
}
