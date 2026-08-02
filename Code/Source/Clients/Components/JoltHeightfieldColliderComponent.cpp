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
                        // No AppearsInAddComponentMenu: EditorJoltHeightfieldColliderComponent owns the
                        // menu entry (PhysX-style editor/runtime split). The runtime component
                        // stays registered for old prefabs and BuildGameEntity.
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::RemoveableByUser, true)
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

        // Seed the mirror the update path compares against, so the first change can be
        // located without fetching the whole grid a second time.
        m_lastHeights = heights;

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
        const AZ::Aabb& dirtyRegion,
        Physics::HeightfieldProviderNotifications::HeightfieldChangeMask changeMask)
    {
        if ((changeMask & Physics::HeightfieldProviderNotifications::HeightfieldChangeMask::HeightData) !=
            Physics::HeightfieldProviderNotifications::HeightfieldChangeMask::None)
        {
            // The provider says exactly what changed; taking it means a shovel-sized
            // deformation costs a shovel-sized update rather than the whole map.
            RefreshHeightsFromProvider(dirtyRegion);
        }
    }

    void JoltHeightfieldColliderComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        // A safety net for providers that change data without firing
        // HeightfieldProviderNotificationBus, throttled because it is not free: the only
        // way to ask "did anything change?" is to fetch the entire grid and compare it,
        // so at 60 fps this was megabytes of copy and compare per second per heightfield
        // to usually learn that nothing had happened. Every fifteenth frame bounds how
        // stale a silent change can get - about a quarter second - at a fifteenth of the
        // cost.
        if (--m_ticksUntilProviderPoll > 0)
        {
            return;
        }
        m_ticksUntilProviderPoll = TicksBetweenProviderPolls;
        RefreshHeightsFromProvider(AZ::Aabb::CreateNull());
    }

    void JoltHeightfieldColliderComponent::RefreshHeightsFromProvider(const AZ::Aabb& dirtyRegion)
    {
        if (!m_nativeShape)
        {
            return;
        }

        size_t numColumns = 0;
        size_t numRows = 0;
        Physics::HeightfieldProviderRequestsBus::EventResult(
            numColumns, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeightfieldGridColumns);
        Physics::HeightfieldProviderRequestsBus::EventResult(
            numRows, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeightfieldGridRows);
        if (numColumns < 2 || numRows < 2)
        {
            return;
        }

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

        // Sample bounds of what actually changed, as a half-open [min, max) grid box.
        size_t changedMinColumn = 0;
        size_t changedMinRow = 0;
        size_t changedMaxColumn = numColumns;
        size_t changedMaxRow = numRows;

        const bool mirrorIsUsable = m_lastHeights.size() == numColumns * numRows;
        bool haveRegion = false;
        if (dirtyRegion.IsValid() && mirrorIsUsable)
        {
            size_t startColumn = 0;
            size_t startRow = 0;
            size_t regionColumns = 0;
            size_t regionRows = 0;
            Physics::HeightfieldProviderRequestsBus::Event(
                GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeightfieldIndicesFromRegion,
                dirtyRegion, startColumn, startRow, regionColumns, regionRows);

            if (regionColumns > 0 && regionRows > 0 && startColumn < numColumns && startRow < numRows)
            {
                changedMinColumn = startColumn;
                changedMinRow = startRow;
                changedMaxColumn = AZStd::min(startColumn + regionColumns, numColumns);
                changedMaxRow = AZStd::min(startRow + regionRows, numRows);

                // Read back only the region, into the mirror. This is the provider API's
                // own partial path - the gem used to ignore it and re-fetch everything.
                AZStd::vector<float>& mirror = m_lastHeights;
                Physics::HeightfieldProviderRequestsBus::Event(
                    GetEntityId(), &Physics::HeightfieldProviderRequests::UpdateHeightsAndMaterials,
                    [&mirror, numColumns](size_t column, size_t row, const Physics::HeightMaterialPoint& point)
                    {
                        mirror[row * numColumns + column] = point.m_height;
                    },
                    startColumn, startRow, changedMaxColumn - startColumn, changedMaxRow - startRow);
                haveRegion = true;
            }
        }

        if (!haveRegion)
        {
            // No usable region: fetch the grid and find the change ourselves. Still better
            // than pushing everything to Jolt, since the sub-rectangle below is what gets
            // recompressed.
            AZStd::vector<float> heights;
            Physics::HeightfieldProviderRequestsBus::EventResult(
                heights, GetEntityId(), &Physics::HeightfieldProviderRequests::GetHeights);
            if (heights.size() != numColumns * numRows)
            {
                return;
            }
            if (heights == m_lastHeights)
            {
                return; // nothing changed since the last look
            }

            if (mirrorIsUsable)
            {
                changedMinColumn = numColumns;
                changedMinRow = numRows;
                changedMaxColumn = 0;
                changedMaxRow = 0;
                for (size_t row = 0; row < numRows; ++row)
                {
                    for (size_t column = 0; column < numColumns; ++column)
                    {
                        const size_t index = row * numColumns + column;
                        if (heights[index] != m_lastHeights[index])
                        {
                            changedMinColumn = AZStd::min(changedMinColumn, column);
                            changedMinRow = AZStd::min(changedMinRow, row);
                            changedMaxColumn = AZStd::max(changedMaxColumn, column + 1);
                            changedMaxRow = AZStd::max(changedMaxRow, row + 1);
                        }
                    }
                }
                if (changedMinColumn >= changedMaxColumn || changedMinRow >= changedMaxRow)
                {
                    return;
                }
            }
            m_lastHeights = AZStd::move(heights);
        }

        auto* joltSystem = GetJoltSystem();
        if (!joltSystem || !joltSystem->GetJoltAllocator())
        {
            return;
        }

        // Jolt requires the rectangle to sit on block boundaries, so grow it outwards to
        // the enclosing blocks. Samples past the provider's grid are the no-collision
        // padding the shape was built with.
        const AZ::u32 blockSize = AZStd::max(heightFieldShape->GetBlockSize(), 1u);
        const auto snapDown = [blockSize](size_t value)
        {
            return static_cast<AZ::u32>((value / blockSize) * blockSize);
        };
        const auto snapUp = [blockSize, sampleCount](size_t value)
        {
            const AZ::u32 snapped = static_cast<AZ::u32>(((value + blockSize - 1) / blockSize) * blockSize);
            return AZStd::min(snapped, sampleCount);
        };

        const AZ::u32 blockMinColumn = snapDown(changedMinColumn);
        const AZ::u32 blockMinRow = snapDown(changedMinRow);
        const AZ::u32 blockMaxColumn = snapUp(changedMaxColumn);
        const AZ::u32 blockMaxRow = snapUp(changedMaxRow);
        const AZ::u32 blockColumns = blockMaxColumn - blockMinColumn;
        const AZ::u32 blockRows = blockMaxRow - blockMinRow;
        if (blockColumns == 0 || blockRows == 0)
        {
            return;
        }

        AZStd::vector<float> patch(static_cast<size_t>(blockColumns) * blockRows, FLT_MAX /* cNoCollisionValue */);
        for (AZ::u32 row = 0; row < blockRows; ++row)
        {
            const size_t sourceRow = blockMinRow + row;
            if (sourceRow >= numRows)
            {
                continue;
            }
            for (AZ::u32 column = 0; column < blockColumns; ++column)
            {
                const size_t sourceColumn = blockMinColumn + column;
                if (sourceColumn >= numColumns)
                {
                    continue;
                }
                patch[static_cast<size_t>(row) * blockColumns + column] =
                    m_lastHeights[sourceRow * numColumns + sourceColumn];
            }
        }

        heightFieldShape->SetHeights(
            blockMinColumn, blockMinRow, blockColumns, blockRows, patch.data(),
            static_cast<intptr_t>(blockColumns), *joltSystem->GetJoltAllocator());

        // Refresh the static body's broadphase bounds.
        AzPhysics::SimulatedBody* simulatedBody = nullptr;
        AzPhysics::SimulatedBodyComponentRequestsBus::EventResult(
            simulatedBody, GetEntityId(), &AzPhysics::SimulatedBodyComponentRequestsBus::Events::GetSimulatedBody);
        auto* staticBody = azrtti_cast<JoltStaticRigidBody*>(simulatedBody);
        if (!staticBody)
        {
            return;
        }
        auto* scene = static_cast<JoltScene*>(staticBody->GetScene());
        auto* bodyInterface = scene ? scene->GetBodyInterface() : nullptr;
        if (!bodyInterface)
        {
            return;
        }

        const AZ::Vector3 comPosition = staticBody->GetPosition();
        bodyInterface->NotifyShapeChanged(
            staticBody->GetBodyId(),
            JPH::Vec3(comPosition.GetX(), comPosition.GetY(), comPosition.GetZ()),
            false /* update mass properties */,
            JPH::EActivation::DontActivate);

        // Wake dynamic bodies over the part that moved: shifting the surface under a
        // sleeping body generates no contacts on its own in Jolt. Scoped to the changed
        // region when the provider named one - waking every body on the terrain because
        // one corner of it deformed is a scene-wide cost for a local event.
        //
        // Note: Jolt's heightfield narrowphase only visits blocks whose height range
        // overlaps the body's AABB, so raising the surface *past* a body in one step
        // strands it underneath. Providers should animate large raises in small
        // increments so bodies can ride the surface up.
        const AZ::Aabb wakeRegion = dirtyRegion.IsValid() ? dirtyRegion : staticBody->GetAabb();
        for (const auto& [crc, body] : scene->GetSimulatedBodyList())
        {
            if (body && body != staticBody && body->GetAabb().Overlaps(wakeRegion))
            {
                if (auto* rigidBody = azdynamic_cast<AzPhysics::RigidBody*>(body))
                {
                    rigidBody->ForceAwake();
                }
            }
        }
    }

} // namespace JoltPhysics
