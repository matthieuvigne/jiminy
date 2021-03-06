#include <algorithm>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "jiminy/core/Utilities.h"
#include "jiminy/core/Model.h"
#include "jiminy/core/BasicMotors.h"


namespace jiminy
{
    SimpleMotor::SimpleMotor(std::string const & name) :
    AbstractMotorBase(name),
    motorOptions_(nullptr)
    {
        /* AbstractMotorBase constructor calls the base implementations of
           the virtual methods since the derived class is not available at
           this point. Thus it must be called explicitly in the constructor. */
        setOptions(getDefaultOptions());
    }

    result_t SimpleMotor::initialize(std::string const & jointName)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!isAttached_)
        {
            std::cout << "Error - SimpleMotor::initialize - Motor not attached to any model. Impossible to initialize it." << std::endl;
            returnCode = result_t::ERROR_GENERIC;
        }

        if (returnCode == result_t::SUCCESS)
        {
            jointName_ = jointName;
            isInitialized_ = true;
        }

        if (returnCode == result_t::SUCCESS)
        {
            returnCode = refreshProxies();
        }

        return returnCode;
    }

    result_t SimpleMotor::setOptions(configHolder_t motorOptions)
    {
        result_t returnCode = result_t::SUCCESS;

        returnCode = AbstractMotorBase::setOptions(motorOptions);

        // Check if the friction parameters make sense
        if (returnCode == result_t::SUCCESS)
        {
            // Make sure the user-defined position limit has the right dimension
            if(boost::get<float64_t>(motorOptions.at("frictionViscousPositive")) > 0.0)
            {
                std::cout << "Error - SimpleMotor::setOptions - 'frictionViscousPositive' must be negative." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
            if(boost::get<float64_t>(motorOptions.at("frictionViscousNegative")) > 0.0)
            {
                std::cout << "Error - SimpleMotor::setOptions - 'frictionViscousNegative' must be negative." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
            if(boost::get<float64_t>(motorOptions.at("frictionDryPositive")) > 0.0)
            {
                std::cout << "Error - SimpleMotor::setOptions - 'frictionDryPositive' must be negative." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
            if(boost::get<float64_t>(motorOptions.at("frictionDryNegative")) > 0.0)
            {
                std::cout << "Error - SimpleMotor::setOptions - 'frictionDryNegative' must be negative." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
            if(boost::get<float64_t>(motorOptions.at("frictionDrySlope")) < 0.0)
            {
                std::cout << "Error - SimpleMotor::setOptions - 'frictionDrySlope' must be positive." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
        }

        if (returnCode == result_t::SUCCESS)
        {
            motorOptions_ = std::make_unique<motorOptions_t const>(motorOptions);
        }

        return returnCode;
    }

    result_t SimpleMotor::computeEffort(float64_t const & t,
                                        float64_t const & q,
                                        float64_t const & v,
                                        float64_t const & a,
                                        float64_t const & uCommand)
    {
        if (!isInitialized_)
        {
            std::cout << "Error - SimpleMotor::computeEffort - Motor not initialized. Impossible to compute actual motor torque." << std::endl;
            return result_t::ERROR_INIT_FAILED;
        }

        // Bypass
        data() = uCommand;

        // Enforce the torque limits
        if (motorOptions_->enableTorqueLimit)
        {
            data() = clamp(data(), -getTorqueLimit(), getTorqueLimit());
        }

        // Add friction to the joints associated with the motor if enable
        if (motorOptions_->enableFriction)
        {
            float64_t const & vMotor = v;
            if (vMotor > 0)
            {
                data() += motorOptions_->frictionViscousPositive * vMotor
                       + motorOptions_->frictionDryPositive * tanh(motorOptions_->frictionDrySlope * vMotor);
            }
            else
            {
                data() += motorOptions_->frictionViscousNegative * vMotor
                       + motorOptions_->frictionDryNegative * tanh(motorOptions_->frictionDrySlope * vMotor);
            }
        }

        return result_t::SUCCESS;
    }
}
