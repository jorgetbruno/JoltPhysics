#pragma once

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Physics/Shape.h>

#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

namespace JoltPhysics
{
    //! Editor-side base for Jolt collider components (mirrors the PhysX gem's
    //! editor/runtime component split). Derives from EditorComponentBase so it
    //! activates in the Edit viewport (unlike the plain AZ::Component runtime
    //! colliders, which only activate in the Game Entity Context), draws the
    //! collider shape there, and spawns the runtime collider via BuildGameEntity.
    class EditorJoltColliderComponentBase
        : public AzToolsFramework::Components::EditorComponentBase
        , private AzFramework::EntityDebugDisplayEventBus::Handler
    {
    public:
        AZ_RTTI(EditorJoltColliderComponentBase, "{A1B2C3D4-E5F6-4789-90AB-CDEF01234567}", AzToolsFramework::Components::EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

    protected:
        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // AzFramework::EntityDebugDisplayEvents
        void DisplayEntityViewport(
            const AzFramework::ViewportInfo& viewportInfo,
            AzFramework::DebugDisplayRequests& debugDisplay) override;

        //! Derived classes draw their shape in world space.
        virtual void DrawShape(AzFramework::DebugDisplayRequests& debugDisplay) const = 0;

        Physics::ColliderConfiguration& GetColliderConfiguration()
        {
            return *m_colliderConfiguration;
        }
        const Physics::ColliderConfiguration& GetColliderConfiguration() const
        {
            return *m_colliderConfiguration;
        }

        AZStd::shared_ptr<Physics::ColliderConfiguration> m_colliderConfiguration =
            AZStd::make_shared<Physics::ColliderConfiguration>();
    };
} // namespace JoltPhysics
