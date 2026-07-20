#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <AzCore/JSON/rapidjson.h>
#include <AzCore/JSON/prettywriter.h>
#include <AzCore/JSON/stringbuffer.h>
#include <AzCore/Serialization/Json/JsonSerialization.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/Components/JoltBoxColliderComponent.h>
#include <Clients/Components/JoltRigidBodyComponent.h>

namespace JoltPhysics
{
    // Verifies that the Jolt components serialize/deserialize correctly through the
    // AZ JSON serialization used by prefabs. The test environment reflects the gem's
    // component descriptors into the application serialize context, same as the editor.
    class JoltComponentSerializationTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            AZ::ComponentApplicationBus::BroadcastResult(
                m_serializeContext, &AZ::ComponentApplicationRequests::GetSerializeContext);
            AZ_Assert(m_serializeContext != nullptr, "No application serialize context");
        }

        template<typename T>
        AZStd::string StoreToJson(const T& object)
        {
            rapidjson::Document doc;
            AZ::JsonSerializerSettings settings;
            settings.m_serializeContext = m_serializeContext;
            settings.m_keepDefaults = true;
            auto result = AZ::JsonSerialization::Store(doc, doc.GetAllocator(), object, settings);
            EXPECT_TRUE(result.GetProcessing() != AZ::JsonSerializationResult::Processing::Halted)
                << result.ToString("root").c_str();

            rapidjson::StringBuffer buffer;
            rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
            doc.Accept(writer);
            return AZStd::string(buffer.GetString(), buffer.GetSize());
        }

        AZ::SerializeContext* m_serializeContext = nullptr;
    };

    TEST_F(JoltComponentSerializationTests, BoxColliderContainsColliderAndShapeConfiguration)
    {
        JoltBoxColliderComponent component;
        AZStd::string json = StoreToJson(component);

        EXPECT_NE(json.find("ColliderConfiguration"), AZStd::string::npos);
        EXPECT_NE(json.find("ShapeConfiguration"), AZStd::string::npos);
    }

    TEST_F(JoltComponentSerializationTests, BoxColliderRoundTripPreservesValues)
    {
        JoltBoxColliderComponent source;
        // Set non-default values through reflection to verify they survive the round trip.
        source.GetShapeColliderPair().first->m_position = AZ::Vector3(1.0f, 2.0f, -0.5f);
        auto* boxShape = static_cast<Physics::BoxShapeConfiguration*>(source.GetShapeColliderPair().second.get());
        boxShape->m_dimensions = AZ::Vector3(4.0f, 5.0f, 6.0f);

        AZStd::string json = StoreToJson(source);

        JoltBoxColliderComponent loaded;
        rapidjson::Document doc;
        doc.Parse(json.c_str(), json.size());
        ASSERT_FALSE(doc.HasParseError());

        AZ::JsonDeserializerSettings settings;
        settings.m_serializeContext = m_serializeContext;
        auto result = AZ::JsonSerialization::Load(loaded, doc, settings);
        EXPECT_TRUE(result.GetProcessing() != AZ::JsonSerializationResult::Processing::Halted)
            << result.ToString("root").c_str();

        EXPECT_EQ(loaded.GetShapeColliderPair().first->m_position.GetZ(), -0.5f);
        auto* loadedBoxShape = static_cast<Physics::BoxShapeConfiguration*>(loaded.GetShapeColliderPair().second.get());
        EXPECT_EQ(loadedBoxShape->m_dimensions, AZ::Vector3(4.0f, 5.0f, 6.0f));
    }

    TEST_F(JoltComponentSerializationTests, RigidBodyRoundTripPreservesValues)
    {
        JoltRigidBodyComponent source;
        source.GetConfiguration().m_mass = 42.0f;
        source.GetConfiguration().m_kinematic = true;

        AZStd::string json = StoreToJson(source);
        EXPECT_NE(json.find("RigidBodyConfiguration"), AZStd::string::npos);

        JoltRigidBodyComponent loaded;
        rapidjson::Document doc;
        doc.Parse(json.c_str(), json.size());
        ASSERT_FALSE(doc.HasParseError());

        AZ::JsonDeserializerSettings settings;
        settings.m_serializeContext = m_serializeContext;
        auto result = AZ::JsonSerialization::Load(loaded, doc, settings);
        EXPECT_TRUE(result.GetProcessing() != AZ::JsonSerializationResult::Processing::Halted)
            << result.ToString("root").c_str();

        EXPECT_EQ(loaded.GetConfiguration().m_mass, 42.0f);
        EXPECT_TRUE(loaded.GetConfiguration().m_kinematic);
    }

} // namespace JoltPhysics
