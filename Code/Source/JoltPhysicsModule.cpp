#include <JoltPhysicsModule.h>

#include <Clients/JoltPhysicsSystemComponent.h>
#include <Clients/ComponentDescriptors.h>
#include <Configuration/JoltSettingsRegistryManager.h>
#include <System/JoltSystem.h>

#if defined(JOLT_EDITOR)
#include <Editor/JoltPhysicsEditorSystemComponent.h>
#include <Editor/EditorComponentDescriptors.h>
#include <Editor/JoltEditorSettingsRegistryManager.h>
#endif

namespace JoltPhysics
{
    JoltPhysicsModule::JoltPhysicsModule()
        : AZ::Module()
    {
        static_assert(alignof(JoltPhysics::JoltSystemConfiguration) == 16);

        LoadModules();

#if defined(JOLT_EDITOR)
        auto registryManager = AZStd::make_unique<JoltEditorSettingsRegistryManager>();
#else
        auto registryManager = AZStd::make_unique<JoltSettingsRegistryManager>();
#endif
        m_joltSystem = AZStd::make_unique<JoltSystem>(AZStd::move(registryManager));

        AZStd::list<AZ::ComponentDescriptor*> descriptorsToAdd = GetDescriptors();
        m_descriptors.insert(m_descriptors.end(), descriptorsToAdd.begin(), descriptorsToAdd.end());

#if defined(JOLT_EDITOR)
        AZStd::list<AZ::ComponentDescriptor*> editorDescriptorsToAdd = GetEditorDescriptors();
        m_descriptors.insert(m_descriptors.end(), editorDescriptorsToAdd.begin(), editorDescriptorsToAdd.end());
#endif
    }

    JoltPhysicsModule::~JoltPhysicsModule()
    {
        if (m_joltSystem)
        {
            m_joltSystem->Shutdown();
            m_joltSystem.reset();
        }

        AZ::GetGlobalSerializeContextModule().Cleanup();

        UnloadModules();
    }

    AZ::ComponentTypeList JoltPhysicsModule::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<JoltPhysicsSystemComponent>(),
#if defined(JOLT_EDITOR)
            azrtti_typeid<JoltPhysicsEditorSystemComponent>(),
#endif
        };
    }

    void JoltPhysicsModule::LoadModules()
    {
#if defined(JOLT_EDITOR)
        {
            AZStd::unique_ptr<AZ::DynamicModuleHandle> sceneCoreModule = AZ::DynamicModuleHandle::Create("SceneCore");
            [[maybe_unused]] bool ok = sceneCoreModule->Load(AZ::DynamicModuleHandle::LoadFlags::InitFuncRequired);
            AZ_Error("JoltPhysics::JoltPhysicsModule", ok, "Error loading SceneCore module");

            m_modules.push_back(AZStd::move(sceneCoreModule));
        }
#endif
    }

    void JoltPhysicsModule::UnloadModules()
    {
        for (auto module = m_modules.rbegin(); module != m_modules.rend(); ++module)
        {
            module->reset();
        }
        m_modules.clear();
    }
} // namespace JoltPhysics

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), JoltPhysics::JoltPhysicsModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_JoltPhysics, JoltPhysics::JoltPhysicsModule)
#endif
