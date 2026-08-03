#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Math/Vector3.h>

namespace JoltPhysics
{
    //! Bus for Jolt-specific character gameplay not covered by Physics::CharacterRequestBus:
    //! ground state, and the gravity the character falls under.
    //!
    //! PhysX puts the gravity half in a separate CharacterGameplayComponent that a project
    //! opts into. Here it lives on the character controller itself, because that component
    //! already owns the velocity contract this has to add to, and because Jolt's character
    //! reports its own ground state - there is no second component's worth of work left to
    //! justify one.
    class JoltCharacterGameplayRequests
        : public AZ::ComponentBus
    {
    public:
        //! Returns whether the character is currently standing on the ground.
        virtual bool IsOnGround() const = 0;

        //! Returns the surface normal of the ground the character is standing on.
        virtual AZ::Vector3 GetGroundNormal() const = 0;

        //! How much of the scene's gravity this character feels. 1 is normal weight, 0
        //! turns gravity off for an animation-driven character that supplies its own
        //! vertical motion, and larger or negative values are open to whoever wants them.
        virtual float GetGravityMultiplier() const = 0;
        virtual void SetGravityMultiplier(float gravityMultiplier) = 0;

        //! The velocity gravity has built up, which the character keeps between frames.
        //!
        //! Writable because a jump is exactly this: set an upward velocity and let it be
        //! eaten away. It also covers handing control back after an animation has moved
        //! the character itself - set the velocity the animation ended on, or the
        //! character resumes falling from a standstill.
        virtual AZ::Vector3 GetFallingVelocity() const = 0;
        virtual void SetFallingVelocity(const AZ::Vector3& fallingVelocity) = 0;
    };
    using JoltCharacterGameplayRequestBus = AZ::EBus<JoltCharacterGameplayRequests>;
} // namespace JoltPhysics
