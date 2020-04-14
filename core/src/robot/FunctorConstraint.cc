#include "jiminy/core/robot/Robot.h"
#include "jiminy/core/Utilities.h"

#include "jiminy/core/robot/FunctorConstraint.h"


namespace jiminy
{
    FunctorConstraint::FunctorConstraint(jacobianFunctor_t jacobianFunctor,
                                         driftFunctor_t driftFunctor,
                                         uint32_t const & constraintSize) :
    AbstractConstraint(),
    jacobianFunctor_(jacobianFunctor),
    driftFunctor_(driftFunctor),
    constraintSize_(constraintSize)
    {
        // Empty on purpose
    }

    FunctorConstraint::~FunctorConstraint(void)
    {
        // Empty on purpose
    }

    void FunctorConstraint::setConstraintSize(uint32_t const& size)
    {
        constraintSize_ = size;
        // Resize the jacobian / drift.
        if (isAttached_)
        {
            jacobian_ = matrixN_t::Zero(constraintSize_, model_->pncModel_.nv);
            drift_ = vectorN_t::Zero(constraintSize_);
        }
    }

    matrixN_t const & FunctorConstraint::getJacobian(Eigen::Ref<vectorN_t const> const & q)
    {
        if (isAttached_)
        {
            jacobianFunctor_(q, jacobian_);
        }
        return jacobian_;
    }

    vectorN_t const & FunctorConstraint::getDrift(Eigen::Ref<vectorN_t const> const & q,
                                                  Eigen::Ref<vectorN_t const> const & v)
    {
        if (isAttached_)
        {
            driftFunctor_(q, v, drift_);
        }
        return drift_;
    }

    hresult_t FunctorConstraint::attach(Model const * model)
    {
        if (isAttached_)
        {
            std::cout << "Error - FunctorConstraint::attach - Constraint already attached to a robot." << std::endl;
            return hresult_t::ERROR_GENERIC;
        }
        model_ = model;
        // Refresh proxies: this sets the matrix and the drift to the right size.
        hresult_t returnCode = refreshProxies();
        if (returnCode == hresult_t::SUCCESS)
        {
             isAttached_ = true;
        }
        return returnCode;
    }

    hresult_t FunctorConstraint::refreshProxies()
    {
        jacobian_ = matrixN_t::Zero(constraintSize_, model_->pncModel_.nv);
        drift_ = vectorN_t::Zero(constraintSize_);
        return hresult_t::SUCCESS;
    }
}


