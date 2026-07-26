#include <AzTest/AzTest.h>

#include <AzCore/JSON/document.h>
#include <AzCore/JSON/stringbuffer.h>
#include <AzCore/JSON/writer.h>
#include <AzCore/Serialization/Json/JsonSerialization.h>

#include <Editor/Components/EditorJoltBoxColliderComponent.h>

namespace JoltPhysics
{
    //! The editor colliders used to hold their configurations behind AZStd::shared_ptr;
    //! they are value members now. AZ's JSON serializer stores a shared_ptr's pointee
    //! transparently, so data saved by the old layout must load into the new one
    //! unchanged - this pins that, using a payload in exactly the shape the old
    //! components wrote into prefabs.
    class JoltEditorColliderSerializationTests : public ::testing::Test
    {
    protected:
        //! Serializes without keeping defaults, the way prefabs are written.
        static rapidjson::Document Store(const EditorJoltBoxColliderComponent& component)
        {
            rapidjson::Document doc;
            AZ::JsonSerializerSettings settings;
            settings.m_keepDefaults = false;
            settings.m_reporting =
                [](AZStd::string_view message, AZ::JsonSerializationResult::ResultCode result, AZStd::string_view path)
            {
                if (result.GetProcessing() == AZ::JsonSerializationResult::Processing::Halted)
                {
                    printf("HALT at '%.*s': %.*s\n", int(path.size()), path.data(), int(message.size()), message.data());
                }
                return result;
            };
            const auto result = AZ::JsonSerialization::Store(doc, doc.GetAllocator(), component, settings);
            EXPECT_NE(AZ::JsonSerializationResult::Processing::Halted, result.GetProcessing())
                << result.ToString("").c_str();
            return doc;
        }
    };

    TEST_F(JoltEditorColliderSerializationTests, SharedPtrEraJsonLoadsIntoValueMembers)
    {
        // Captured from a prefab written while the members were shared_ptrs.
        const char* sharedPtrEraJson = R"JSON({
            "ColliderConfiguration": {
                "CollisionLayer": { "Index": 3 },
                "CollisionGroupId": { "GroupId": "{7B4D2E90-58C1-4A63-9F2B-6E0A3D8C15F7}" },
                "Trigger": true,
                "Position": [1.0, 2.0, 3.0]
            },
            "ShapeConfiguration": {
                "Scale": [1.0, 1.0, 1.0],
                "Configuration": [2.0, 4.0, 6.0]
            }
        })JSON";

        rapidjson::Document doc;
        ASSERT_FALSE(doc.Parse(sharedPtrEraJson).HasParseError());

        EditorJoltBoxColliderComponent component;
        const auto result = AZ::JsonSerialization::Load(component, doc);
        ASSERT_NE(AZ::JsonSerializationResult::Processing::Halted, result.GetProcessing());

        // Verified through a re-store rather than accessors: what matters is that the
        // values survive into what the prefab system would write next.
        const rapidjson::Document out = Store(component);

        ASSERT_TRUE(out.HasMember("ColliderConfiguration"));
        const auto& collider = out["ColliderConfiguration"];
        EXPECT_EQ(3, collider["CollisionLayer"]["Index"].GetInt());
        EXPECT_STREQ("{7B4D2E90-58C1-4A63-9F2B-6E0A3D8C15F7}", collider["CollisionGroupId"]["GroupId"].GetString());
        EXPECT_TRUE(collider["Trigger"].GetBool());
        EXPECT_FLOAT_EQ(3.0f, collider["Position"][2].GetFloat());

        ASSERT_TRUE(out.HasMember("ShapeConfiguration"));
        EXPECT_FLOAT_EQ(2.0f, out["ShapeConfiguration"]["Configuration"][0].GetFloat());
        EXPECT_FLOAT_EQ(6.0f, out["ShapeConfiguration"]["Configuration"][2].GetFloat());
    }

    TEST_F(JoltEditorColliderSerializationTests, DefaultComponentStoresWhatTheSharedPtrLayoutStored)
    {
        // A default component stores exactly what the shared_ptr layout stored: the
        // "Entire object" material slot (ColliderConfiguration's constructor differs
        // from a field-default MaterialSlots there) and nothing else. More would mean
        // every freshly added collider carries override noise into its prefab; less
        // would mean the value-member conversion changed the on-disk shape.
        const EditorJoltBoxColliderComponent component;
        const rapidjson::Document out = Store(component);

        ASSERT_TRUE(out.HasMember("ColliderConfiguration"));
        const auto& collider = out["ColliderConfiguration"];
        EXPECT_EQ(1u, collider.MemberCount());
        ASSERT_TRUE(collider.HasMember("MaterialSlots"));
        EXPECT_STREQ("Entire object", collider["MaterialSlots"]["Slots"][0]["Name"].GetString());

        EXPECT_FALSE(out.HasMember("ShapeConfiguration"));
    }

} // namespace JoltPhysics
