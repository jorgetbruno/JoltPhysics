#include <Clients/Components/JoltHeightfieldColliderComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/ColliderComponentBus.h>
#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>

#include <RigidBody/JoltStaticRigidBody.h>
#include <Scene/JoltScene.h>
#include <Shape/JoltHeightfieldUtils.h>
#include <System/JoltSystem.h>

#include <Jolt/Jolt.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>

namespace JoltPhysics
{
    void JoltHeightfieldColliderComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltHeightfieldColliderComponent, JoltColliderComponentBase>()
                ->Version(1)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltHeightfieldColliderComponent>(
                    "Jolt Heightfield Collider",
                    "Heightfield collider fed by a Physics::HeightfieldProviderBus implementation (e.g. a terrain gem)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void JoltHeightfieldColliderComponent::Activate()
    {
        BuildHeightfieldShape();
        JoltColliderComponentBase::Activate();
        Physics::HeightfieldProviderNotificationBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
    }

    void JoltHeightfieldColliderComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        Physics::HeightfieldProviderNotificationBus::Handler::BusDisconnect();
        m_shapeConfiguration->SetCachedNativeHeightfield(nullptr);
        m_nativeShape = nullptr;
        JoltColliderComponentBase::Deactivate();
    }

    AzPhysics::ShapeColliderPair JoltHeightfieldColliderComponent::GetShapeColliderPair() const
    {
        return { m_colliderConfiguration, m_shapeConfiguration };
    }

    bool JoltHeightfieldColliderComponent::BuildHeightfieldShape()
    {
        AZ::Vector2 gridSpacing;
        Physics::HeightfieldProviderRequestsBus::EventResult(gridSpacing, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeightfieldGridSpacing);

        size_t numColumns = 0;
        size_t numRows = 0;
        Physics::HeightfieldProviderRequestsBus::EventResult(
            numColumns, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeightfieldGridColumns);
        Physics::HeightfieldProviderRequestsBus::EventResult(
            numRows, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeightfieldGridRows);

        AZStd::vector<float> heights;
        Physics::HeightfieldProviderRequestsBus::EventResult(heights, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeights);

        if (numColumns < 2 || numRows < 2 || heights.empty())
        {
            return false;
        }

        // Per-triangle material slots; Jolt resolves friction/restitution at contact time
        // (see JoltStaticRigidBody::CreateInScene), so default Jolt materials suffice here.
        AZStd::vector<AZ::Data::Asset<Physics::MaterialAsset>> providerMaterials;
        Physics::HeightfieldProviderRequestsBus::EventResult(providerMaterials, GetEntityId(), &Physics::HeightfieldProviderRequests::GetMaterialList);

        JPH::PhysicsMaterialList nativeMaterials;
        for (size_t i = 0; i < AZStd::max<size_t>(providerMaterials.size(), 1); ++i)
        {
            nativeMaterials.push_back(JPH::PhysicsMaterial::sDefault);
        }

        AZStd::vector<Physics::HeightMaterialPoint> heightsAndMaterials;
        Physics::HeightfieldProviderRequestsBus::EventResult(heightsAndMaterials, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeightsAndMaterials);

        AZStd::vector<AZ::u8> materialIndices;
        if (!heightsAndMaterials.empty())
        {
            materialIndices.reserve(heightsAndMaterials.size());
            for (const auto& point : heightsAndMaterials)
            {
                materialIndices.push_back(point.m_materialIndex);
            }
        }

        m_nativeShape = JoltHeightfieldUtils::CreateHeightFieldShape(
            static_cast<AZ::u32>(numColumns),
            static_cast<AZ::u32>(numRows),
            gridSpacing,
            heights,
            materialIndices,
            nativeMaterials);

        if (m_nativeShape)
        {
            // The cached pointer on the shape configuration is released independently
            // (via JoltPhysicsSystemComponent::ReleaseNativeHeightfieldObject, triggered by
            // SetCachedNativeHeightfield whenever the cached pointer changes/clears - see
            // Deactivate() and the rebuild path below). Take an extra reference here so that
            // release doesn't fight with this component's own m_nativeShape lifetime.
            const_cast<JPH::Shape*>(m_nativeShape.GetPtr())->AddRef();
        }
        m_shapeConfiguration->SetCachedNativeHeightfield(const_cast<JPH::Shape*>(m_nativeShape.GetPtr()));

        return m_nativeShape != nullptr;
    }

    void JoltHeightfieldColliderComponent::OnHeightfieldDataChanged(
        [[maybe_unused]] const AZ::Aabb& dirtyRegion,
        Physics::HeightfieldProviderNotifications::HeightfieldChangeMask changeMask)
    {
        if ((changeMask & Physics::HeightfieldProviderNotifications::HeightfieldChangeMask::HeightData) !=
            Physics::HeightfieldProviderNotifications::HeightfieldChangeMask::None)
        {
            UpdateHeightsFromProvider();
        }
    }

    void JoltHeightfieldColliderComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        // Poll as a safety net for providers that change data without firing
        // HeightfieldProviderNotificationBus (UpdateHeightsFromProvider early-outs
        // when nothing changed).
        UpdateHeightsFromProvider();
    }

    void JoltHeightfieldColliderComponent::UpdateHeightsFromProvider()
    {
        if (!m_nativeShape)
        {
            return;
        }

        AZ::Vector2 gridSpacing;
        Physics::HeightfieldProviderRequestsBus::EventResult(gridSpacing, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeightfieldGridSpacing);

        size_t numColumns = 0;
        size_t numRows = 0;
        Physics::HeightfieldProviderRequestsBus::EventResult(
            numColumns, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeightfieldGridColumns);
        Physics::HeightfieldProviderRequestsBus::EventResult(
            numRows, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeightfieldGridRows);

        AZStd::vector<float> heights;
        Physics::HeightfieldProviderRequestsBus::EventResult(heights, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeights);

        if (numColumns < 2 || numRows < 2 || heights.size() != numColumns * numRows)
        {
            return;
        }

        if (heights == m_lastHeights)
        {
            // Nothing changed since the last update.
            return;
        }
        m_lastHeights = heights;

        auto* heightFieldShape = static_cast<JPH::HeightFieldShape*>(const_cast<JPH::Shape*>(m_nativeShape.GetPtr()));
        const AZ::u32 sampleCount = heightFieldShape->GetSampleCount();
        if (AZStd::max(numColumns, numRows) > sampleCount)
        {
            // Grid grew beyond the current shape; rebuild instead.
            BuildHeightfieldShape();
            Physics::ColliderComponentEventBus::Event(
                GetEntityId(), &Physics::ColliderComponentEvents::OnColliderChanged);
            return;
        }

        AZStd::vector<float> paddedHeights(sampleCount * sampleCount, FLT_MAX /* cNoCollisionValue */);
        for (size_t y = 0; y < numRows; ++y)
        {
            for (size_t x = 0; x < numColumns; ++x)
            {
                paddedHeights[y * sampleCount + x] = heights[y * numColumns + x];
            }
        }

        auto* joltSystem = GetJoltSystem();
        if (!joltSystem || !joltSystem->GetJoltAllocator())
        {
            return;
        }

        heightFieldShape->SetHeights(
            0, 0, sampleCount, sampleCount, paddedHeights.data(), static_cast<intptr_t>(sampleCount), *joltSystem->GetJoltAllocator());

        // Refresh the static body's broadphase bounds.
        AzPhysics::SimulatedBody* simulatedBody = nullptr;
        AzPhysics::SimulatedBodyComponentRequestsBus::EventResult(
            simulatedBody, GetEntityId(), &AzPhysics::SimulatedBodyComponentRequestsBus::Events::GetSimulatedBody);
        if (auto* staticBody = azrtti_cast<JoltStaticRigidBody*>(simulatedBody))
        {
            if (auto* scene = static_cast<JoltScene*>(staticBody->GetScene()))
            {
                if (auto* bodyInterface = scene->GetBodyInterface())
                {
                    const AZ::Vector3 comPosition = staticBody->GetPosition();
                    bodyInterface->NotifyShapeChanged(
                        staticBody->GetBodyId(),
                        JPH::Vec3(comPosition.GetX(), comPosition.GetY(), comPosition.GetZ()),
                        false /* update mass properties */,
                        JPH::EActivation::DontActivate);

                    // Wake dynamic bodies overlapping the heightfield: moving the surface
                    // under a sleeping body generates no contacts on its own in Jolt.
                    // Note: Jolt's heightfield narrowphase only visits blocks whose height
                    // range overlaps the body's AABB, so raising the surface *past* a body
                    // in one step strands it underneath. Providers should animate large
                    // raises in small increments so bodies can ride the surface up.
                    const AZ::Aabb heightfieldAabb = staticBody->GetAabb();
                    for (const auto& [crc, body] : scene->GetSimulatedBodyList())
                    {
                        if (body && body != staticBody && body->GetAabb().Overlaps(heightfieldAabb))
                        {
                            if (auto* rigidBody = azdynamic_cast<AzPhysics::RigidBody*>(body))
                            {
                                rigidBody->ForceAwake();
                            }
                        }
                    }
                }
            }
        }
    }

} // namespace JoltPhysics
