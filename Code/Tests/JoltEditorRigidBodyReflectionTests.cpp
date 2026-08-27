/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>

#include <Editor/Components/EditorJoltRigidBodyComponent.h>
#include <Clients/Components/JoltRigidBodyComponent.h>

namespace JoltPhysics
{
    //! The editor never instantiates the runtime component, so a setting reflected only
    //! there is invisible and unreachable however important it is. Interpolate motion was
    //! exactly that: a body could be interpolating with nothing in the inspector saying so,
    //! and no way to turn it off short of editing the prefab by hand.
    class EditorRigidBodyReflectionTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            m_serializeContext = AZStd::make_unique<AZ::SerializeContext>();
            m_serializeContext->CreateEditContext();
            JoltRigidBodyComponent::Reflect(m_serializeContext.get());
            EditorJoltRigidBodyComponent::Reflect(m_serializeContext.get());
        }

        void TearDown() override
        {
            m_serializeContext->DestroyEditContext();
            m_serializeContext.reset();
        }

        AZStd::unique_ptr<AZ::SerializeContext> m_serializeContext;
    };

    TEST_F(EditorRigidBodyReflectionTests, InterpolateMotionIsReachableFromTheEditorComponent)
    {
        AZ::EditContext* editContext = m_serializeContext->GetEditContext();
        ASSERT_NE(editContext, nullptr);

        const AZ::SerializeContext::ClassData* classData =
            m_serializeContext->FindClassData(azrtti_typeid<EditorJoltRigidBodyComponent>());
        ASSERT_NE(classData, nullptr);
        ASSERT_NE(classData->m_editData, nullptr);

        bool found = false;
        for (const AZ::Edit::ElementData& element : classData->m_editData->m_elements)
        {
            if (element.m_name != nullptr && AZStd::string_view(element.m_name) == "Interpolate motion")
            {
                found = true;
                break;
            }
        }
        EXPECT_TRUE(found) << "the editor component must expose Interpolate motion; reflecting it only on the "
                              "runtime component leaves it unreachable in the inspector";
    }

    TEST_F(EditorRigidBodyReflectionTests, BuildGameEntityCarriesTheInterpolationChoice)
    {
        EditorJoltRigidBodyComponent editorComponent;
        editorComponent.SetMotionInterpolationForTesting(JoltMotionInterpolation::Off);

        AZ::Entity gameEntity("GameEntity");
        editorComponent.BuildGameEntity(&gameEntity);

        auto* runtimeComponent = gameEntity.FindComponent<JoltRigidBodyComponent>();
        ASSERT_NE(runtimeComponent, nullptr);
        EXPECT_EQ(runtimeComponent->GetMotionInterpolation(), JoltMotionInterpolation::Off)
            << "the choice must travel to the runtime component, or turning it off does nothing";
    }
} // namespace JoltPhysics
