#pragma once

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>

namespace JoltPhysics
{
    class JoltSystem;
    class JoltSettingsRegistryManager;

    class JoltPhysicsModule : public AZ::Module
    {
    public:
        AZ_RTTI(JoltPhysicsModule, "{B5C7D9E1-F3A5-7B9C-1D3E-5F7A9B1C3D5E}", AZ::Module);
        AZ_CLASS_ALLOCATOR(JoltPhysicsModule, AZ::SystemAllocator);

        JoltPhysicsModule();
        ~JoltPhysicsModule() override;

        AZ::ComponentTypeList GetRequiredSystemComponents() const override;

    private:
        void LoadModules();
        void UnloadModules();

        AZStd::unique_ptr<JoltSystem> m_joltSystem;
        AZStd::vector<AZStd::unique_ptr<AZ::DynamicModuleHandle>> m_modules;
    };

} // namespace JoltPhysics
