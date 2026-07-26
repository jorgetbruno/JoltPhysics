#include <Editor/Components/EditorJoltDistanceJointComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltJointComponents.h>
#include <Editor/Components/EditorJoltDebugDrawUtils.h>
#include <Joint/JoltJointConfiguration.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void EditorJoltDistanceJointComponent::Reflect(AZ::ReflectContext* context)
    {
        EditorJoltJointComponentBase::Reflect(context);
        Internal::ReflectOnce<JoltDistanceJointConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltDistanceJointComponent, EditorJoltJointComponentBase>()
                ->Version(1)
                ->Field("GenericProperties", &EditorJoltDistanceJointComponent::m_genericProperties)
                ->Field("MinDistance", &EditorJoltDistanceJointComponent::m_minDistance)
                ->Field("MaxDistance", &EditorJoltDistanceJointComponent::m_maxDistance)
                ->Field("SpringFrequency", &EditorJoltDistanceJointComponent::m_springFrequency)
                ->Field("SpringDamping", &EditorJoltDistanceJointComponent::m_springDamping)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltDistanceJointComponent>(
                    "Jolt Distance Joint", "Distance joint simulated by the Jolt physics backend (editor)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltDistanceJointComponent::m_genericProperties,
                        "Generic properties", "Break force/torque and generic joint flags.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltDistanceJointComponent::m_minDistance,
                        "Min distance", "Minimum separation between the two attachment points.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltDistanceJointComponent::m_maxDistance,
                        "Max distance", "Maximum separation between the two attachment points.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltDistanceJointComponent::m_springFrequency,
                        "Spring frequency", "Spring oscillation frequency; 0 = hard limits.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " Hz")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltDistanceJointComponent::m_springDamping,
                        "Spring damping", "Spring damping ratio (used when frequency > 0).")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ;
            }
        }
    }

    void EditorJoltDistanceJointComponent::Activate()
    {
        EditorJoltJointComponentBase::Activate();
        ConnectJointComponentMode<EditorJoltDistanceJointComponent>();
    }

    void EditorJoltDistanceJointComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltDistanceJointComponent>())
        {
            component->GetConfiguration() = m_configuration;
            component->GetGenericProperties() = m_genericProperties;
            component->SetDistanceParams(m_minDistance, m_maxDistance, m_springFrequency, m_springDamping);
        }
    }

    void EditorJoltDistanceJointComponent::DrawJointLimits(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& jointTransform) const
    {
        // A distance joint constrains how far the two anchors may separate, so the
        // limits are spheres about the joint frame rather than a cone or an arc.
        EditorDebugDraw::DrawWireSphere(
            debugDisplay, jointTransform, m_maxDistance, EditorDebugDraw::LimitColor);

        if (m_minDistance > 0.0f)
        {
            EditorDebugDraw::DrawWireSphere(
                debugDisplay, jointTransform, m_minDistance, EditorDebugDraw::LimitColor);
        }
    }

} // namespace JoltPhysics
