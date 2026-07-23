#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Math/Vector3.h>

namespace JoltPhysics
{
    //! Bus for Jolt-specific character gameplay queries not covered by Physics::CharacterRequestBus.
    class JoltCharacterGameplayRequests
        : public AZ::ComponentBus
    {
    public:
        //! Returns whether the character is currently standing on the ground.
        virtual bool IsOnGround() const = 0;

        //! Returns the surface normal of the ground the character is standing on.
        virtual AZ::Vector3 GetGroundNormal() const = 0;
    };
    using JoltCharacterGameplayRequestBus = AZ::EBus<JoltCharacterGameplayRequests>;
} // namespace JoltPhysics
