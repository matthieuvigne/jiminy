///////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief      Class for defining a constraint externally using jacobian and drift-computing functions.
///
/// \details    This class creates a constraint from two user-given function defining the jacobian
///             and the drift.
///
///////////////////////////////////////////////////////////////////////////////////////////////

#ifndef JIMINY_FUNCTOR_CONSTRAINT_H
#define JIMINY_FUNCTOR_CONSTRAINT_H

#include <memory>

#include "jiminy/core/Types.h"
#include "jiminy/core/robot/AbstractConstraint.h"


namespace jiminy
{
    class Model;


    using jacobianFunctor_t = std::function<void(Eigen::Ref<vectorN_t const> const & /*q*/,
                                                 matrixN_t & /*jacobianOut*/)>;
    using driftFunctor_t = std::function<void(Eigen::Ref<vectorN_t const> const & /*q*/,
                                              Eigen::Ref<vectorN_t const> const & /*v*/,
                                              vectorN_t & /*driftOut*/)>;

    /// \brief Create a constraint from a user-defined function computing the jacobian and the drift.
    ///
    /// \details The user supplies a jacobian-computing function (and likewise a drif-computing function)
    ///          with signature void jacobianFunctor(vectorN_t const & q, matrixN_t & jacobianOut)
    ///          While using a reference to an internal buffer allows for performance improvement, this
    ///          requires the user to define in advance the size (dimension, i.e. number of lines)
    ///          of the constraint. Note also that the jacobian is not overwritten between each call:
    ///          it is up to the user to reset this matrix to zero if needed.
    class FunctorConstraint: public AbstractConstraint
    {

    public:
        ///////////////////////////////////////////////////////////////////////////////////////////////
        /// \brief      Forbid the copy of the class
        ///////////////////////////////////////////////////////////////////////////////////////////////
        FunctorConstraint(FunctorConstraint const & ) = delete;
        FunctorConstraint & operator = (FunctorConstraint const & ) = delete;

        ///////////////////////////////////////////////////////////////////////////////////////////////
        /// \brief      Constructor
        ///
        /// \param[in]  jacobianFunctor Function for computing the jacobian.
        /// \param[in]  driftFunctor Function for computing the drift.
        /// \param[in]  constraintSize Dimension of the constraint.
        ///////////////////////////////////////////////////////////////////////////////////////////////
        FunctorConstraint(jacobianFunctor_t jacobianFunctor,
                          driftFunctor_t driftFunctor,
                          uint32_t const & size);
        virtual ~FunctorConstraint(void);

        /// \brief Set the size of the constraint, resizing the buffers as needed.
        /// \param[in] size Size of the constraint (i.e. number of rows / elements of the jacobian / drift).
        void setConstraintSize(uint32_t const & size);

        ///////////////////////////////////////////////////////////////////////////////////////////////
        /// \brief    Compute and return the jacobian of the constraint.
        ///
        /// \note     To avoid duplicate kinematic computation, it is assumed that
        ///           computeJointJacobians and framesForwardKinematics has already
        ///           been called on model->pncModel_.
        ///
        /// \param[in] q    Current joint position.
        /// \return         Jacobian of the constraint.
        ///////////////////////////////////////////////////////////////////////////////////////////////
        virtual matrixN_t const & getJacobian(Eigen::Ref<vectorN_t const> const & q) override final;

        ///////////////////////////////////////////////////////////////////////////////////////////////
        /// \brief    Compute and return the drift of the constraint.
        ///
        /// \note     To avoid duplicate kinematic computation, it is assumed that forward kinematics
        ///           and jacobian computation has already been done on model->pncModel_.
        ///
        /// \param[in] q    Current joint position.
        /// \param[in] v    Current joint velocity.
        /// \return         Drift of the constraint.
        ///////////////////////////////////////////////////////////////////////////////////////////////
        virtual vectorN_t const & getDrift(Eigen::Ref<vectorN_t const>  const & q,
                                           Eigen::Ref<vectorN_t const>  const & v) override final;

    protected:
        ///////////////////////////////////////////////////////////////////////////////////////////////
        /// \brief      Link the constraint on the given model, and initialize it.
        ///
        /// \param[in] model    Model on which to apply the constraint.
        /// \return     Error code: attach may fail if:
        ///              - the constraint is already attached.
        ///              - the target frame name does not exist in model.
        ///////////////////////////////////////////////////////////////////////////////////////////////
        virtual hresult_t attach(Model const * model) override final;

        ///////////////////////////////////////////////////////////////////////////////////////////////
        /// \brief    Refresh the proxies.
        ///
        /// \remark   This method is not intended to be called manually. The Robot to which the
        ///           motor is added is taking care of it when its own `refresh` method is called.
        ///////////////////////////////////////////////////////////////////////////////////////////////
        virtual hresult_t refreshProxies(void) override final;

    private:
        jacobianFunctor_t jacobianFunctor_; ///< Function computing the jacobian.
        driftFunctor_t driftFunctor_; ///< Function computing the drift.
        uint32_t constraintSize_; ///< Dimension of the constraint.
    };
}

#endif //end of JIMINY_ABSTRACT_MOTOR_H
