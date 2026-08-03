#include <Clients/Components/JoltShapeColliderComponent.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Math/PolygonPrism.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <AzFramework/Physics/ColliderComponentBus.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <LmbrCentral/Shape/BoxShapeComponentBus.h>
#include <LmbrCentral/Shape/CapsuleShapeComponentBus.h>
#include <LmbrCentral/Shape/CylinderShapeComponentBus.h>
#include <LmbrCentral/Shape/PolygonPrismShapeComponentBus.h>
#include <LmbrCentral/Shape/SphereShapeComponentBus.h>

#include <Shape/JoltCylinderShapeConfiguration.h>
#include <Shape/JoltMeshUtils.h>
#include <Utils/ReflectionUtils.h>

namespace JoltPhysics
{
    void JoltShapeColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        // ReflectOnce, like the sibling colliders: every collider component reflects the
        // shared base, and registering it twice is a duplicate-Uuid error.
        Internal::ReflectOnce<JoltColliderComponentBase>(context);
        Internal::ReflectOnce<Physics::ShapeConfiguration>(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltShapeColliderComponent, JoltColliderComponentBase>()->Version(1);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltShapeColliderComponent>("Jolt Shape Collider",
                    "Collision geometry taken from the shape component on this entity")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        // The editor component owns the Add Component entry, as with every
                        // other collider in this gem.
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::RemoveableByUser, true)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void JoltShapeColliderComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        JoltColliderComponentBase::GetRequiredServices(required);
        // The whole point of this collider is that something else supplies its geometry.
        required.push_back(AZ_CRC_CE("ShapeService"));
    }

    void JoltShapeColliderComponent::Activate()
    {
        LmbrCentral::ShapeComponentNotificationsBus::Handler::BusConnect(GetEntityId());
        JoltColliderComponentBase::Activate();
    }

    void JoltShapeColliderComponent::Deactivate()
    {
        JoltColliderComponentBase::Deactivate();
        LmbrCentral::ShapeComponentNotificationsBus::Handler::BusDisconnect();
    }

    void JoltShapeColliderComponent::OnShapeChanged([[maybe_unused]] ShapeChangeReasons changeReasons)
    {
        // Resizing the shape resizes the collider; the body rebuilds from the new pair.
        Physics::ColliderComponentEventBus::Event(
            GetEntityId(), &Physics::ColliderComponentEvents::OnColliderChanged);
    }

    AzPhysics::ShapeColliderPair JoltShapeColliderComponent::GetShapeColliderPair() const
    {
        return { AZStd::make_shared<Physics::ColliderConfiguration>(
                     const_cast<JoltShapeColliderComponent*>(this)->GetColliderConfiguration()),
                 BuildShapeConfigurationForEntity(GetEntityId()) };
    }

    AZStd::shared_ptr<Physics::ShapeConfiguration> JoltShapeColliderComponent::BuildShapeConfigurationForEntity(
        AZ::EntityId entityId)
    {
        AZ::Crc32 shapeType;
        LmbrCentral::ShapeComponentRequestsBus::EventResult(
            shapeType, entityId, &LmbrCentral::ShapeComponentRequests::GetShapeType);

        if (shapeType == AZ_CRC_CE("Box"))
        {
            AZ::Vector3 dimensions = AZ::Vector3::CreateOne();
            LmbrCentral::BoxShapeComponentRequestsBus::EventResult(
                dimensions, entityId, &LmbrCentral::BoxShapeComponentRequests::GetBoxDimensions);
            return AZStd::make_shared<Physics::BoxShapeConfiguration>(dimensions);
        }

        if (shapeType == AZ_CRC_CE("Sphere"))
        {
            float radius = 0.5f;
            LmbrCentral::SphereShapeComponentRequestsBus::EventResult(
                radius, entityId, &LmbrCentral::SphereShapeComponentRequests::GetRadius);
            return AZStd::make_shared<Physics::SphereShapeConfiguration>(radius);
        }

        if (shapeType == AZ_CRC_CE("Capsule"))
        {
            float height = 1.0f;
            float radius = 0.25f;
            LmbrCentral::CapsuleShapeComponentRequestsBus::EventResult(
                height, entityId, &LmbrCentral::CapsuleShapeComponentRequests::GetHeight);
            LmbrCentral::CapsuleShapeComponentRequestsBus::EventResult(
                radius, entityId, &LmbrCentral::CapsuleShapeComponentRequests::GetRadius);
            return AZStd::make_shared<Physics::CapsuleShapeConfiguration>(height, radius);
        }

        if (shapeType == AZ_CRC_CE("Cylinder"))
        {
            float height = 1.0f;
            float radius = 0.5f;
            LmbrCentral::CylinderShapeComponentRequestsBus::EventResult(
                height, entityId, &LmbrCentral::CylinderShapeComponentRequests::GetHeight);
            LmbrCentral::CylinderShapeComponentRequestsBus::EventResult(
                radius, entityId, &LmbrCentral::CylinderShapeComponentRequests::GetRadius);

            auto cylinder = AZStd::make_shared<JoltCylinderShapeConfiguration>();
            cylinder->m_height = height;
            cylinder->m_radius = radius;
            return cylinder;
        }

        if (shapeType == AZ_CRC_CE("PolygonPrism"))
        {
            return BuildPolygonPrismConfiguration(entityId);
        }

        AZ_Warning("JoltPhysics", false,
            "Jolt Shape Collider on entity %s: its shape component is a type this collider does not wrap "
            "(Box, Sphere, Capsule, Cylinder and Polygon Prism are supported). The entity has no collision "
            "geometry.",
            entityId.ToString().c_str());
        return nullptr;
    }

    AZStd::shared_ptr<Physics::ShapeConfiguration> JoltShapeColliderComponent::BuildPolygonPrismConfiguration(
        AZ::EntityId entityId)
    {
        AZ::PolygonPrismPtr prism;
        LmbrCentral::PolygonPrismShapeComponentRequestBus::EventResult(
            prism, entityId, &LmbrCentral::PolygonPrismShapeComponentRequests::GetPolygonPrism);
        if (!prism)
        {
            return nullptr;
        }

        const AZStd::vector<AZ::Vector2>& footprint = prism->m_vertexContainer.GetVertices();
        if (footprint.size() < 3)
        {
            AZ_Warning("JoltPhysics", false,
                "Jolt Shape Collider on entity %s: the polygon prism has fewer than three vertices, so there is "
                "nothing to extrude.",
                entityId.ToString().c_str());
            return nullptr;
        }

        // The prism is an outline extruded along local +Z, so its solid is the hull of the
        // outline at both heights. Jolt builds the hull from the point cloud itself, so no
        // triangulation is needed here.
        AZStd::vector<AZ::Vector3> points;
        points.reserve(footprint.size() * 2);
        const float height = prism->GetHeight();
        for (const AZ::Vector2& vertex : footprint)
        {
            points.emplace_back(vertex.GetX(), vertex.GetY(), 0.0f);
            points.emplace_back(vertex.GetX(), vertex.GetY(), height);
        }

        const AZStd::vector<AZ::u8> cooked =
            JoltMeshUtils::PackConvexMesh(points.data(), static_cast<AZ::u32>(points.size()));
        if (cooked.empty())
        {
            return nullptr;
        }

        auto cookedConfig = AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
        cookedConfig->SetCookedMeshData(
            cooked.data(), cooked.size(), Physics::CookedMeshShapeConfiguration::MeshType::Convex);
        return cookedConfig;
    }
} // namespace JoltPhysics
