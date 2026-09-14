#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <JoltEditorRecordingDebugDisplay.h>

#include <Editor/Components/EditorJoltForceRegionComponent.h>
#include <Editor/Components/EditorJoltForceRegionDraw.h>
#include <ForceRegion/JoltForceRegionComponent.h>

namespace JoltPhysics
{
    // A fan, a current or a wind zone used to be a wireframe box with nothing in it saying
    // which way it pushed; the only way to find out was to enter game mode and drop
    // something in. The editor component exists to draw that, and to give the runtime
    // component the same editor/runtime split every other body-level component has.
    class JoltEditorForceRegionTests : public ::testing::Test
    {
    protected:
        static constexpr float Length = EditorDebugDraw::ForceArrowLength;

        static AZStd::shared_ptr<JoltForceWorldSpace> WorldForce(const AZ::Vector3& direction, float magnitude)
        {
            auto force = AZStd::make_shared<JoltForceWorldSpace>();
            force->m_direction = direction;
            force->m_magnitude = magnitude;
            return force;
        }

        static AZStd::shared_ptr<JoltForceLocalSpace> LocalForce(const AZ::Vector3& direction, float magnitude)
        {
            auto force = AZStd::make_shared<JoltForceLocalSpace>();
            force->m_direction = direction;
            force->m_magnitude = magnitude;
            return force;
        }

        static AZStd::shared_ptr<JoltForcePoint> PointForce(float magnitude)
        {
            auto force = AZStd::make_shared<JoltForcePoint>();
            force->m_magnitude = magnitude;
            return force;
        }

        //! A serialize context with an edit context, with both components reflected in
        //! the given order. Order is the point: the region's own reflection is shared,
        //! and whichever component reflects second must not register it again.
        static AZStd::unique_ptr<AZ::SerializeContext> ReflectBoth(bool editorFirst)
        {
            auto serializeContext = AZStd::make_unique<AZ::SerializeContext>();
            serializeContext->CreateEditContext();
            if (editorFirst)
            {
                EditorJoltForceRegionComponent::Reflect(serializeContext.get());
                JoltForceRegionComponent::Reflect(serializeContext.get());
            }
            else
            {
                JoltForceRegionComponent::Reflect(serializeContext.get());
                EditorJoltForceRegionComponent::Reflect(serializeContext.get());
            }
            return serializeContext;
        }

        static bool EditContextExposes(AZ::SerializeContext& serializeContext, const AZ::Uuid& typeId, const char* elementName)
        {
            const AZ::SerializeContext::ClassData* classData = serializeContext.FindClassData(typeId);
            if (classData == nullptr || classData->m_editData == nullptr)
            {
                return false;
            }
            for (const AZ::Edit::ElementData& element : classData->m_editData->m_elements)
            {
                if (element.m_name != nullptr && AZStd::string_view(element.m_name) == elementName)
                {
                    return true;
                }
            }
            return false;
        }

        RecordingDebugDisplay m_display;
    };

    TEST_F(JoltEditorForceRegionTests, TheForcesAndTheWindTagAreReachableFromTheEditorComponent)
    {
        // The editor never instantiates the runtime component, so a field reflected only
        // there is invisible and unreachable however important it is.
        const AZStd::unique_ptr<AZ::SerializeContext> serializeContext = ReflectBoth(false);
        EXPECT_TRUE(EditContextExposes(*serializeContext, azrtti_typeid<EditorJoltForceRegionComponent>(), "Forces"));
        EXPECT_TRUE(EditContextExposes(*serializeContext, azrtti_typeid<EditorJoltForceRegionComponent>(), "Wind tag"));
        serializeContext->DestroyEditContext();
    }

    TEST_F(JoltEditorForceRegionTests, ReflectingTheEditorComponentFirstIsHarmless)
    {
        // Both components reflect JoltForceRegion. Descriptor order is not something a
        // component gets to choose, and the second unguarded Class<JoltForceRegion>()
        // asserts - which shows up here as a failure, not a crash.
        const AZStd::unique_ptr<AZ::SerializeContext> serializeContext = ReflectBoth(true);
        EXPECT_NE(serializeContext->FindClassData(azrtti_typeid<JoltForceRegion>()), nullptr);
        EXPECT_NE(serializeContext->FindClassData(azrtti_typeid<JoltForceRegionComponent>()), nullptr);
        EXPECT_NE(serializeContext->FindClassData(azrtti_typeid<EditorJoltForceRegionComponent>()), nullptr);
        serializeContext->DestroyEditContext();
    }

    TEST_F(JoltEditorForceRegionTests, BuildGameEntityCarriesTheForcesAndTheWindTag)
    {
        JoltForceRegion region;
        region.m_forces.push_back(WorldForce(AZ::Vector3(0.0f, 1.0f, 0.0f), 8.0f));
        region.m_forces.push_back(PointForce(-3.0f));

        EditorJoltForceRegionComponent editorComponent;
        editorComponent.SetForceRegionForTesting(region);
        editorComponent.SetWindTagForTesting("global_wind");

        AZ::Entity gameEntity("GameEntity");
        editorComponent.BuildGameEntity(&gameEntity);

        auto* runtimeComponent = gameEntity.FindComponent<JoltForceRegionComponent>();
        ASSERT_NE(runtimeComponent, nullptr);
        EXPECT_EQ(runtimeComponent->GetWindTag(), "global_wind")
            << "the tag must travel, or a wind zone authored in the editor blows nothing";
        ASSERT_EQ(runtimeComponent->GetForceRegion().m_forces.size(), 2u);

        // The forces themselves, not just the count: the same net force out of the same
        // question. The wind vector is the world-space force's own, so it is the readout.
        EXPECT_TRUE(runtimeComponent->GetForceRegion().GetWindVelocity().IsClose(AZ::Vector3(0.0f, 8.0f, 0.0f), 1e-4f));
    }

    // ---- the preview ----------------------------------------------------------------

    TEST_F(JoltEditorForceRegionTests, AWorldSpaceForcePointsAlongItsAxisWhateverTheRegionDoes)
    {
        JoltForceRegion region;
        region.m_forces.push_back(WorldForce(AZ::Vector3(1.0f, 0.0f, 0.0f), 10.0f));

        // Rotated a quarter turn about Z: a world-space force must not turn with it.
        const AZ::Vector3 origin(2.0f, 3.0f, 4.0f);
        const AZ::Transform rotated = AZ::Transform::CreateFromQuaternionAndTranslation(
            AZ::Quaternion::CreateRotationZ(AZ::Constants::HalfPi), origin);
        EditorDebugDraw::DrawForceRegionPreview(m_display, region, rotated);

        EXPECT_TRUE(m_display.HasSegment(origin, origin + AZ::Vector3(Length, 0.0f, 0.0f)))
            << "the shaft of a +X world force is not drawn along world +X";
    }

    TEST_F(JoltEditorForceRegionTests, ALocalSpaceForceTurnsWithTheRegion)
    {
        JoltForceRegion region;
        region.m_forces.push_back(LocalForce(AZ::Vector3(1.0f, 0.0f, 0.0f), 10.0f));

        const AZ::Vector3 origin(2.0f, 3.0f, 4.0f);
        const AZ::Transform rotated = AZ::Transform::CreateFromQuaternionAndTranslation(
            AZ::Quaternion::CreateRotationZ(AZ::Constants::HalfPi), origin);
        EditorDebugDraw::DrawForceRegionPreview(m_display, region, rotated);

        // Local +X under a quarter turn about Z is world +Y - exactly what
        // JoltForceLocalSpace::CalculateForce would push along.
        EXPECT_TRUE(m_display.HasSegment(origin, origin + AZ::Vector3(0.0f, Length, 0.0f)))
            << "the shaft of a local +X force did not turn with the region";
        EXPECT_FALSE(m_display.HasSegment(origin, origin + AZ::Vector3(Length, 0.0f, 0.0f)))
            << "a local-space force was drawn as if it were world-space";
    }

    TEST_F(JoltEditorForceRegionTests, ANegativeMagnitudeReversesTheArrow)
    {
        JoltForceRegion region;
        region.m_forces.push_back(WorldForce(AZ::Vector3(0.0f, 0.0f, 1.0f), -10.0f));

        const AZ::Vector3 origin = AZ::Vector3::CreateZero();
        EditorDebugDraw::DrawForceRegionPreview(m_display, region, AZ::Transform::CreateIdentity());

        EXPECT_TRUE(m_display.HasSegment(origin, AZ::Vector3(0.0f, 0.0f, -Length)))
            << "a negative magnitude pushes the other way, and the arrow must say so";
    }

    TEST_F(JoltEditorForceRegionTests, APointForceDrawsOutwardForARepulsorAndInwardForAWell)
    {
        const AZ::Vector3 origin(1.0f, 1.0f, 1.0f);
        const AZ::Transform at = AZ::Transform::CreateTranslation(origin);

        {
            JoltForceRegion repulsor;
            repulsor.m_forces.push_back(PointForce(5.0f));
            RecordingDebugDisplay display;
            EditorDebugDraw::DrawForceRegionPreview(display, repulsor, at);
            for (const AZ::Vector3& axis :
                 { AZ::Vector3::CreateAxisX(), -AZ::Vector3::CreateAxisX(), AZ::Vector3::CreateAxisY(),
                   -AZ::Vector3::CreateAxisY(), AZ::Vector3::CreateAxisZ(), -AZ::Vector3::CreateAxisZ() })
            {
                EXPECT_TRUE(display.HasSegment(origin, origin + axis * Length))
                    << "a repulsor is missing its outward arrow along " << axis.GetX() << "," << axis.GetY() << "," << axis.GetZ();
            }
        }
        {
            JoltForceRegion well;
            well.m_forces.push_back(PointForce(-5.0f));
            RecordingDebugDisplay display;
            EditorDebugDraw::DrawForceRegionPreview(display, well, at);
            for (const AZ::Vector3& axis :
                 { AZ::Vector3::CreateAxisX(), -AZ::Vector3::CreateAxisX(), AZ::Vector3::CreateAxisY(),
                   -AZ::Vector3::CreateAxisY(), AZ::Vector3::CreateAxisZ(), -AZ::Vector3::CreateAxisZ() })
            {
                EXPECT_TRUE(display.HasSegment(origin + axis * Length, origin))
                    << "a well is missing its inward arrow along " << axis.GetX() << "," << axis.GetY() << "," << axis.GetZ();
            }
            EXPECT_FALSE(display.HasSegment(origin, origin + AZ::Vector3(Length, 0.0f, 0.0f)))
                << "a well was drawn pushing outward";
        }
    }

    TEST_F(JoltEditorForceRegionTests, ForcesWithNoDirectionOfTheirOwnDrawNothing)
    {
        // Drag and damping oppose whatever velocity a body arrives with; there is no
        // arrow that would be true before there is a body. A zero magnitude does nothing
        // and so draws nothing, rather than an arrow for a force that is not there.
        JoltForceRegion region;
        region.m_forces.push_back(AZStd::make_shared<JoltForceSimpleDrag>());
        region.m_forces.push_back(AZStd::make_shared<JoltForceLinearDamping>());
        region.m_forces.push_back(WorldForce(AZ::Vector3(1.0f, 0.0f, 0.0f), 0.0f));
        region.m_forces.push_back(WorldForce(AZ::Vector3::CreateZero(), 10.0f));
        region.m_forces.push_back(nullptr);

        EditorDebugDraw::DrawForceRegionPreview(m_display, region, AZ::Transform::CreateIdentity());
        EXPECT_TRUE(m_display.m_segments.empty()) << m_display.m_segments.size() << " segments drawn for forces with nothing to show";
    }
} // namespace JoltPhysics
