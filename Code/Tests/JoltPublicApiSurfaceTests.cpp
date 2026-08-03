// The exported surface of this gem, exercised the way a sibling gem sees it.
//
// The guard here is the include list, not the assertions: this file may include only
// headers that JoltPhysics.API actually publishes (Include/JoltPhysics/...), plus engine
// headers. Anything that drifts back under Source/ - a public header quietly including a
// private one, a bus method taking a private type - breaks this translation unit before
// any test runs, which is the failure an extension gem would otherwise hit instead.

#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <AzCore/Math/Vector3.h>
#include <AzCore/std/smart_ptr/make_shared.h>

#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <AzFramework/Physics/ShapeConfiguration.h>

#include <JoltPhysics/JoltPhysicsBus.h>
#include <JoltPhysics/Pipeline/JoltMeshAsset.h>

namespace JoltPhysics
{
    class JoltPublicApiSurfaceTests : public ::testing::Test
    {
    };

    TEST_F(JoltPublicApiSurfaceTests, TheMeshAssetTypeCanBeNamedAndReadFromOutsideTheGem)
    {
        // Naming the type is the whole ask: an extension gem that wants cooked collision
        // geometry has to be able to declare AZ::Data::Asset<JoltMeshAsset> to load one,
        // and to walk what it loaded. Neither was reachable while the header lived under
        // Source/, so a sibling gem had to re-cook geometry the Asset Processor had
        // already built and deduplicated.
        AZ::Data::Asset<Pipeline::JoltMeshAsset> asset;
        EXPECT_FALSE(asset.GetId().IsValid()) << "a default asset reference should name nothing yet";

        Pipeline::JoltMeshAssetData assetData;

        auto cooked = AZStd::make_shared<Physics::CookedMeshShapeConfiguration>();
        cooked->SetCookedMeshData(nullptr, 0, Physics::CookedMeshShapeConfiguration::MeshType::Convex);
        assetData.m_colliderShapes.emplace_back(nullptr, cooked);
        assetData.m_materialIndexPerShape.push_back(0);
        assetData.m_materialSlots.SetSlots({ "Slot" });

        // The fields a consumer reads to get from an asset to geometry.
        ASSERT_EQ(assetData.m_colliderShapes.size(), 1);
        EXPECT_EQ(assetData.m_colliderShapes[0].second->GetShapeType(), Physics::ShapeType::CookedMesh);
        EXPECT_EQ(assetData.m_materialSlots.GetSlotsCount(), 1);

        // The per-shape overrides the Scene Builder may have stored travel with the shape,
        // and every one of them is optional - the sentinel a consumer checks against.
        Pipeline::JoltAssetColliderConfiguration overrides;
        EXPECT_FALSE(overrides.m_isTrigger.has_value());
        EXPECT_FALSE(overrides.m_transform.has_value());

        // The reserved index that says "the cooked mesh carries its own per-face slots",
        // which a consumer has to recognise to read material indices correctly.
        EXPECT_EQ(Pipeline::JoltMeshAssetData::TriangleMeshMaterialIndex, 0xFFFF);
    }

    TEST_F(JoltPublicApiSurfaceTests, TheSystemBusCarriesTheMeshAssetEntryPoints)
    {
        // JoltPhysics.API is an INTERFACE target with no library behind it, so a helper
        // declared beside the asset type would be a symbol a consumer could not link.
        // Expansion is dispatched instead, and these are the signatures that dispatch
        // has to keep: taking the member pointers fails to compile if either one changes
        // shape or starts naming a type that only exists under Source/.
        using ExpandShapes = AzPhysics::ShapeColliderPairList (JoltPhysicsSystemRequests::*)(
            const Pipeline::JoltMeshAssetData&, const Physics::ColliderConfiguration&, const AZ::Vector3&);
        using ApplySlots = void (JoltPhysicsSystemRequests::*)(
            const Pipeline::JoltMeshAssetData&, bool, Physics::MaterialSlots&);

        constexpr ExpandShapes expandShapes = &JoltPhysicsSystemRequests::GetColliderShapesFromMeshAsset;
        constexpr ApplySlots applySlots = &JoltPhysicsSystemRequests::ApplyMaterialSlotsFromMeshAsset;
        EXPECT_NE(expandShapes, nullptr);
        EXPECT_NE(applySlots, nullptr);

        // And a caller with no handler present gets nothing rather than a crash, which is
        // what an extension gem sees in a project running a different physics backend.
        AzPhysics::ShapeColliderPairList pairs;
        Pipeline::JoltMeshAssetData assetData;
        const Physics::ColliderConfiguration colliderConfiguration;
        JoltPhysicsSystemRequestBus::BroadcastResult(
            pairs,
            &JoltPhysicsSystemRequests::GetColliderShapesFromMeshAsset,
            assetData,
            colliderConfiguration,
            AZ::Vector3::CreateOne());
        EXPECT_TRUE(pairs.empty());
    }
} // namespace JoltPhysics
