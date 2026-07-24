#include <Utils/JoltComponentUtils.h>
#include <Clients/Components/JoltColliderComponentBase.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Color.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/algorithm.h>

#include <AzFramework/Physics/ColliderComponentBus.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JoltColliderComponentBase::Reflect(AZ::ReflectContext* context)
    {
        Internal::ReflectOnce<Physics::ColliderConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->RegisterGenericType<AZStd::shared_ptr<Physics::ColliderConfiguration>>();

            serializeContext->Class<JoltColliderComponentBase, AZ::Component>()
                ->Version(1)
                ->Field("ColliderConfiguration", &JoltColliderComponentBase::m_colliderConfiguration)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltColliderComponentBase>("Jolt Collider Base", "Base configuration for Jolt colliders")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltColliderComponentBase::m_colliderConfiguration,
                        "Collider Configuration", "Configuration shared by all Jolt colliders")
                    ;
            }
        }
    }

    void JoltColliderComponentBase::Activate()
    {
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());

        Physics::ColliderComponentEventBus::Event(
            GetEntityId(), &Physics::ColliderComponentEvents::OnColliderChanged);
    }

    void JoltColliderComponentBase::Deactivate()
    {
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();

        Physics::ColliderComponentEventBus::Event(
            GetEntityId(), &Physics::ColliderComponentEvents::OnColliderChanged);
    }

    void JoltColliderComponentBase::DisplayEntityViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        debugDisplay.SetColor(AZ::Color(0.35f, 0.9f, 0.85f, 1.0f));

        for (const AzPhysics::ShapeColliderPair& pair : GetShapeColliderPairs())
        {
            if (!pair.first || !pair.second)
            {
                continue;
            }
            const Physics::ColliderConfiguration& colliderConfig = *pair.first;
            const Physics::ShapeConfiguration& shapeConfig = *pair.second;

            const AZ::Transform colliderLocalTransform =
                AZ::Transform::CreateFromQuaternionAndTranslation(colliderConfig.m_rotation, colliderConfig.m_position);

            debugDisplay.PushMatrix(worldTransform * colliderLocalTransform);

            switch (shapeConfig.GetShapeType())
            {
            case Physics::ShapeType::Sphere:
                {
                    const auto& sphere = static_cast<const Physics::SphereShapeConfiguration&>(shapeConfig);
                    const float radius = sphere.m_radius * sphere.m_scale.GetX();
                    debugDisplay.DrawWireSphere(AZ::Vector3::CreateZero(), radius);
                    break;
                }

            case Physics::ShapeType::Box:
                {
                    const auto& box = static_cast<const Physics::BoxShapeConfiguration&>(shapeConfig);
                    const AZ::Vector3 halfExtents = (box.m_dimensions * box.m_scale) * 0.5f;
                    debugDisplay.DrawWireBox(-halfExtents, halfExtents);
                    break;
                }

            case Physics::ShapeType::Capsule:
                {
                    const auto& capsule = static_cast<const Physics::CapsuleShapeConfiguration&>(shapeConfig);
                    const float scale = capsule.m_scale.GetX();
                    const float radius = capsule.m_radius * scale;
                    // DrawWireCapsule wants the straight (cylindrical) section height only,
                    // whereas m_height is the total height including both hemispherical caps.
                    const float straightHeight = AZStd::max(0.0f, (capsule.m_height * scale) - (2.0f * radius));
                    debugDisplay.DrawWireCapsule(AZ::Vector3::CreateZero(), AZ::Vector3::CreateAxisZ(), radius, straightHeight);
                    break;
                }

            default:
                // Heightfield, mesh, convex hull, etc: not drawn here.
                break;
            }

            debugDisplay.PopMatrix();
        }
    }

    void JoltColliderComponentBase::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltColliderService"));
    }

    void JoltColliderComponentBase::GetIncompatibleServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        // Multiple colliders per entity are allowed; they are combined into a
        // single compound simulated body (mirrors PhysX BaseColliderComponent).
    }

    void JoltColliderComponentBase::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void JoltColliderComponentBase::OnAfterEntitySet()
    {
        if (m_serializedIdentifier.empty())
        {
            m_serializedIdentifier = Internal::GenerateSerializedIdentifier(this);
        }
    }

    AZStd::string JoltColliderComponentBase::GetSerializedIdentifier() const
    {
        return m_serializedIdentifier;
    }
} // namespace JoltPhysics
