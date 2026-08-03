#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <AzCore/Component/Entity.h>
#include <AzFramework/Components/TransformComponent.h>

#include <AzToolsFramework/ComponentMode/EditorComponentModeBus.h>
#include <AzToolsFramework/Manipulators/ShapeManipulatorRequestBus.h>

#include <Editor/Components/EditorJoltBakedMeshColliderComponent.h>
#include <Editor/Components/EditorJoltBoxColliderComponent.h>
#include <Editor/Components/EditorJoltMeshColliderComponent.h>
#include <Editor/Components/EditorJoltStaticCompoundColliderComponent.h>

namespace JoltPhysics
{
    //! The two halves of "this collider has a draggable offset": a ComponentModeDelegate
    //! connected for the component (the Edit button, and the double-click that enters the
    //! mode), and a ShapeManipulatorRequestBus handler behind it for the handle to move.
    //! The manipulators themselves need a viewport; everything they do to the component
    //! goes through that bus.
    class JoltColliderOffsetComponentModeTests : public ::testing::Test
    {
    protected:
        template<typename ColliderComponentType>
        ColliderComponentType* CreateCollider(const char* name)
        {
            m_entity = AZStd::make_unique<AZ::Entity>(name);
            m_entity->CreateComponent<AzFramework::TransformComponent>();
            auto* component = m_entity->CreateComponent<ColliderComponentType>();
            AZ_Assert(component != nullptr, "collider component could not be created");

            m_entity->Init();
            m_entity->Activate();

            m_idPair = AZ::EntityComponentIdPair(m_entity->GetId(), component->GetId());
            m_collider = component;
            return component;
        }

        void TearDown() override
        {
            if (m_entity)
            {
                m_entity->Deactivate();
                m_entity.reset();
            }
        }

        //! Whether the component offers component mode at all - what the inspector's Edit
        //! button is built from. The delegate answers its own bus only once the entity is
        //! selected in a running editor, so this asks the component instead.
        bool HasComponentMode() const
        {
            return m_collider != nullptr && m_collider->OffersComponentMode();
        }

        AZ::Vector3 GetOffset() const
        {
            AZ::Vector3 offset = AZ::Vector3::CreateZero();
            AzToolsFramework::ShapeManipulatorRequestBus::EventResult(
                offset, m_idPair, &AzToolsFramework::ShapeManipulatorRequests::GetTranslationOffset);
            return offset;
        }

        void SetOffset(const AZ::Vector3& offset)
        {
            AzToolsFramework::ShapeManipulatorRequestBus::Event(
                m_idPair, &AzToolsFramework::ShapeManipulatorRequests::SetTranslationOffset, offset);
        }

        AZStd::unique_ptr<AZ::Entity> m_entity;
        AZ::EntityComponentIdPair m_idPair;
        EditorJoltColliderComponentBase* m_collider = nullptr;
    };

    TEST_F(JoltColliderOffsetComponentModeTests, AnAssetMeshColliderOffersAnOffsetHandle)
    {
        // The offset handler was always here - the base class serves it for every collider
        // - but with no delegate connected there was no Edit button to reach it with, so a
        // shared .joltmesh prop collider could only be nudged by typing coordinates while
        // a box on the same entity had a viewport handle for the identical field.
        CreateCollider<EditorJoltMeshColliderComponent>("AssetMeshCollider");

        EXPECT_TRUE(HasComponentMode()) << "the asset mesh collider offers no component mode";
        ASSERT_TRUE(AzToolsFramework::ShapeManipulatorRequestBus::HasHandlers(m_idPair));

        // What a drag does, in the terms the handle uses.
        const AZ::Vector3 nudged(0.25f, -0.5f, 1.0f);
        SetOffset(nudged);
        EXPECT_TRUE(GetOffset().IsClose(nudged, 1e-4f));
    }

    TEST_F(JoltColliderOffsetComponentModeTests, ABakedMeshColliderOffersAnOffsetHandle)
    {
        // Same gap, same fix: geometry out of a blob is no reason for the offset to be
        // grid-numbers only.
        CreateCollider<EditorJoltBakedMeshColliderComponent>("BakedMeshCollider");

        EXPECT_TRUE(HasComponentMode()) << "the baked mesh collider offers no component mode";
        ASSERT_TRUE(AzToolsFramework::ShapeManipulatorRequestBus::HasHandlers(m_idPair));

        const AZ::Vector3 nudged(-1.5f, 0.0f, 0.75f);
        SetOffset(nudged);
        EXPECT_TRUE(GetOffset().IsClose(nudged, 1e-4f));
    }

    TEST_F(JoltColliderOffsetComponentModeTests, APrimitiveColliderStillOffersItsOwnMode)
    {
        // The primitives reach component mode through the engine's shape modes, which
        // carry a dimensions sub-mode the cooked colliders have nothing to fill. Pinned so
        // that adding an offset-only mode for the mesh colliders cannot quietly become the
        // mode a box uses too.
        CreateCollider<EditorJoltBoxColliderComponent>("BoxCollider");

        EXPECT_TRUE(HasComponentMode());
    }

    TEST_F(JoltColliderOffsetComponentModeTests, ACompoundColliderStillOffersNone)
    {
        // A compound has no geometry of its own - its children are separate entities with
        // colliders of their own - so there is nothing on it to drag, and its offset is
        // not a thing a handle would sit on. Deliberate, and it doubles as proof that the
        // check above can tell a collider with a mode from one without.
        CreateCollider<EditorJoltStaticCompoundColliderComponent>("CompoundCollider");

        EXPECT_FALSE(HasComponentMode());
    }
} // namespace JoltPhysics
