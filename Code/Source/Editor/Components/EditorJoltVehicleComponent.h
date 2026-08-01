#pragma once

#include <Vehicle/JoltVehicleConfiguration.h>

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzToolsFramework/API/ComponentEntitySelectionBus.h>
#include <AzToolsFramework/ComponentMode/ComponentModeDelegate.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

#include <Editor/Components/JoltVehicleWheelRequestBus.h>

namespace JoltPhysics
{
    //! Editor Jolt Vehicle: edit-time counterpart of JoltVehicleComponent.
    //! Spawns the runtime component via BuildGameEntity, copying the vehicle
    //! configuration, and previews the wheels and their suspension travel.
    //!
    //! Wheel positions are also draggable in the viewport: the component serves
    //! JoltVehicleWheelRequestBus and owns the ComponentModeDelegate that puts the Edit
    //! button on it, with the handles themselves in JoltVehicleComponentMode.
    class EditorJoltVehicleComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , private AzFramework::EntityDebugDisplayEventBus::Handler
        , private JoltVehicleWheelRequestBus::Handler
        , private AzToolsFramework::EditorComponentSelectionRequestsBus::Handler
    {
    public:
        AZ_COMPONENT(EditorJoltVehicleComponent, "{D0E1F2A3-B4C5-4678-C9D0-E1F2A3B4C5D6}", AzToolsFramework::Components::EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

        JoltVehicleConfiguration& GetVehicleConfiguration()
        {
            return m_configuration;
        }
        const JoltVehicleConfiguration& GetVehicleConfiguration() const
        {
            return m_configuration;
        }

        //! A wheel's pose after the suspension settle preview, in the entity's frame.
        struct SettledWheel
        {
            AZ::Transform m_localTransform = AZ::Transform::CreateIdentity();
            float m_suspensionLength = 0.0f;
            bool m_onGround = false;
        };

        //! Runs the one-shot suspension settle (the inspector button): simulates the
        //! vehicle for a few seconds in a private throwaway scene - on flat ground at
        //! the height found under the entity in the editor world - and keeps the settled
        //! chassis and wheel poses as a viewport ghost. Public so tests can press the
        //! button; returns the property refresh level like any ChangeNotify handler.
        AZ::u32 OnSettlePreviewPressed();

        bool HasSettlePreview() const
        {
            return m_hasSettlePreview;
        }
        const AZStd::vector<SettledWheel>& GetSettlePreviewWheels() const
        {
            return m_settledWheels;
        }

    private:
        // AzFramework::EntityDebugDisplayEvents
        void DisplayEntityViewport(
            const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        // JoltVehicleWheelRequestBus
        AZ::u32 GetWheelCount() const override;
        AZ::Vector3 GetWheelPosition(AZ::u32 wheelIndex) const override;
        void SetWheelPosition(AZ::u32 wheelIndex, const AZ::Vector3& position) override;
        AZ::Transform GetChassisSpace() const override;

        // AzToolsFramework::EditorComponentSelectionRequestsBus - required by the
        // ComponentModeDelegate, and what the editor picks the vehicle against.
        AZ::Aabb GetEditorSelectionBoundsViewport(const AzFramework::ViewportInfo& viewportInfo) override;
        bool SupportsEditorRayIntersect() override;

        //! A config edit makes the settle ghost stale; drop it.
        AZ::u32 OnConfigurationChanged();

        void RunSettlePreview();

        JoltVehicleConfiguration m_configuration;

        //! Puts the Edit button on the component and enters component mode on double click.
        AzToolsFramework::ComponentModeFramework::ComponentModeDelegate m_componentModeDelegate;

        //! Settle preview results, entity-local so the ghost follows the entity. Not
        //! serialized: a preview is a scratch visualization.
        AZStd::vector<SettledWheel> m_settledWheels;
        AZ::Transform m_settledChassisLocal = AZ::Transform::CreateIdentity();
        bool m_hasSettlePreview = false;
    };
} // namespace JoltPhysics
